using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using StrikerPC.Video;
using LiveKit;
using LiveKit.Proto;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;
using UnityEngine.Networking;

namespace StrikerPC.XR
{
    [Serializable]
    public sealed class StringEvent : UnityEvent<string>
    {
    }

    [Serializable]
    public sealed class SubtitleMessage
    {
        public string type;
        public string id;
        public string text;
        public bool isFinal;
        public string speakerIdentity;
        public string speakerLabel;
        public string language;
        public long timestamp;
        public SubtitleLogEntry entry;
        public SubtitleLogEntry[] entries;
    }

    [Serializable]
    public sealed class SubtitleLogEntry
    {
        public string id;
        public long timestamp;
        public string speaker;
        public string kind;
        public string text;
    }

    public sealed class LiveKitAudioBridge : MonoBehaviour
    {
        [Serializable]
        private sealed class TokenResponse
        {
            public string url;
            public string token;
            public string room;
            public string identity;
            public string role;
            public string error;
        }

        [Serializable]
        private sealed class SubtitleHistoryResponse
        {
            public SubtitleLogEntry[] entries;
        }

        private sealed class RemoteVideoBinding
        {
            public VideoStream Stream;
            public Coroutine Coroutine;
            public VideoStream.TextureReceiveDelegate Handler;
            public bool TargetsPilot;
            public LiveKitVideoSurface ReconnaissanceSurface;
            public string ParticipantIdentity;
            public string TrackName;
        }

        private const string SubtitleTopic = "subtitles";
        private const string SubtitleSyncTopic = "subtitle-sync";
        private const string AccessCodeEnvironmentVariable = "STRIKERPC_LIVEKIT_ACCESS_CODE";
        private const string EndpointEnvironmentVariable = "STRIKERPC_LIVEKIT_TOKEN_ENDPOINT";
        private const string LegacyAccessCodeEnvironmentVariable = "DRONEOPS_LIVEKIT_ACCESS_CODE";
        private const string LegacyEndpointEnvironmentVariable = "DRONEOPS_LIVEKIT_TOKEN_ENDPOINT";
        private const float ReconnectBaseDelaySeconds = 2f;
        private const float ReconnectMaxDelaySeconds = 30f;

        [Header("LiveKit")]
        [SerializeField] private string tokenEndpoint;
        [SerializeField] private string accessCode;
        [SerializeField] private bool connectOnEnable;

        [Header("Quest Audio")]
        [SerializeField] private string preferredMicrophoneNameContains = "Oculus";
        [SerializeField] private AudioSource remoteAudioTemplate;
        [SerializeField] private InputActionProperty microphoneToggleAction;
        [SerializeField] private SubtitleHistoryView subtitleHistoryView;

        [Header("Quest Video")]
        [SerializeField] private string videoPublisherIdentity = "tello-publisher";
        [SerializeField] private PilotVideoSourceSelector videoSourceSelector;
        [SerializeField] private LiveKitVideoSurface reconnaissanceVideoSurface;

        [Header("Events")]
        [SerializeField] private UnityEvent connected;
        [SerializeField] private UnityEvent disconnected;
        [SerializeField] private UnityEvent<bool> microphoneStateChanged;
        [SerializeField] private StringEvent subtitleReceived;
        [SerializeField] private StringEvent statusChanged;

        private readonly Dictionary<string, AudioStream> audioStreams = new Dictionary<string, AudioStream>();
        private readonly Dictionary<string, GameObject> audioObjects = new Dictionary<string, GameObject>();
        private readonly Dictionary<string, RemoteVideoBinding> videoBindings =
            new Dictionary<string, RemoteVideoBinding>();
        private readonly HashSet<string> videoFirstFrames = new HashSet<string>();
        private readonly HashSet<string> subtitleEntryIds = new HashSet<string>();
        private Room room;
        private LocalAudioTrack localAudioTrack;
        private MicrophoneSource microphoneSource;
        private GameObject microphoneObject;
        private bool operationInProgress;
        private bool disconnectRequested;
        private Coroutine reconnectCoroutine;
        private int reconnectAttempt;

        public bool IsConnected => room != null && room.IsConnected;
        public bool IsMicrophoneEnabled => localAudioTrack != null;

        // 설정된 경우 활성화 시 LiveKit 연결을 시작한다.
        private void OnEnable()
        {
            disconnectRequested = false;
            reconnectAttempt = 0;
            subtitleHistoryView?.SetConnectionWaiting(true);
            if (microphoneToggleAction.action == null)
            {
                microphoneToggleAction = new InputActionProperty(new InputAction(
                    "Quest Microphone Toggle",
                    InputActionType.Button,
                    "<XRController>{LeftHand}/primaryButton"));
            }

            if (microphoneToggleAction.action != null)
            {
                microphoneToggleAction.action.performed += OnMicrophoneTogglePerformed;
                microphoneToggleAction.action.Enable();
            }

            if (connectOnEnable)
            {
                Connect();
            }
        }

        // 비활성화 시 Quest 마이크 입력 이벤트를 해제한다.
        private void OnDisable()
        {
            if (microphoneToggleAction.action != null)
            {
                microphoneToggleAction.action.performed -= OnMicrophoneTogglePerformed;
                microphoneToggleAction.action.Disable();
            }
        }

        // Quest 컨트롤러 입력으로 마이크 송출 상태를 전환한다.
        private void OnMicrophoneTogglePerformed(InputAction.CallbackContext context)
        {
            ToggleMicrophone();
        }

        // LiveKit 토큰을 요청하고 Room에 연결한다.
        public void Connect()
        {
            if (operationInProgress || IsConnected)
            {
                return;
            }

            StartCoroutine(ConnectRoutine());
        }

        // LiveKit 연결 과정을 순서대로 수행한다.
        private IEnumerator ConnectRoutine()
        {
            operationInProgress = true;
            statusChanged?.Invoke("LiveKit 토큰 요청 중");

            string resolvedEndpoint = GetEnvironmentVariable(
                EndpointEnvironmentVariable,
                LegacyEndpointEnvironmentVariable);
            if (string.IsNullOrWhiteSpace(resolvedEndpoint))
            {
                resolvedEndpoint = tokenEndpoint;
            }

            if (string.IsNullOrWhiteSpace(resolvedEndpoint))
            {
                statusChanged?.Invoke("LiveKit Token Endpoint가 설정되지 않았습니다.");
                Debug.LogWarning("[LiveKit] Room connect blocked reason=missing-token-endpoint");
                operationInProgress = false;
                ScheduleReconnect();
                yield break;
            }

            string resolvedAccessCode = GetEnvironmentVariable(
                AccessCodeEnvironmentVariable,
                LegacyAccessCodeEnvironmentVariable);
            if (string.IsNullOrWhiteSpace(resolvedAccessCode))
            {
                resolvedAccessCode = accessCode;
            }

            Debug.Log($"[LiveKit] Room connect start endpoint={resolvedEndpoint}");

            using UnityWebRequest request = new UnityWebRequest(resolvedEndpoint, "POST");
            request.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes("{}"));
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");
            request.SetRequestHeader("X-Access-Code", resolvedAccessCode ?? string.Empty);
            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                statusChanged?.Invoke($"LiveKit 토큰 요청 실패: {request.error}");
                Debug.LogWarning(
                    $"[LiveKit] Token request failed result={request.result} error={request.error}");
                operationInProgress = false;
                ScheduleReconnect();
                yield break;
            }

            TokenResponse token = JsonUtility.FromJson<TokenResponse>(
                request.downloadHandler.text);
            if (token == null || string.IsNullOrWhiteSpace(token.url) ||
                string.IsNullOrWhiteSpace(token.token))
            {
                statusChanged?.Invoke(
                    $"LiveKit 토큰 응답 오류: {token?.error ?? "필수 값 없음"}");
                Debug.LogWarning(
                    $"[LiveKit] Token response rejected reason={token?.error ?? "missing-required-fields"}");
                operationInProgress = false;
                ScheduleReconnect();
                yield break;
            }

            room = new Room();
            room.TrackSubscribed += OnTrackSubscribed;
            room.TrackUnsubscribed += OnTrackUnsubscribed;
            room.ParticipantConnected += OnParticipantConnected;
            room.ParticipantDisconnectedWithReason += OnParticipantDisconnected;
            room.DataReceived += OnDataReceived;
            room.Disconnected += OnDisconnected;

            ConnectInstruction connect = room.Connect(
                token.url,
                token.token,
                new LiveKit.RoomOptions());
            yield return connect;
            operationInProgress = false;

            if (connect.IsError)
            {
                statusChanged?.Invoke("LiveKit 연결에 실패했습니다.");
                Debug.LogWarning(
                    $"[LiveKit] Room connect failed room={token.room} identity={token.identity}");
                DisposeRoom();
                ScheduleReconnect();
                yield break;
            }

            reconnectAttempt = 0;
            statusChanged?.Invoke($"LiveKit 연결됨: {token.room} · {token.role}");
            Debug.Log(
                $"[LiveKit] Room connected room={token.room} identity={token.identity} " +
                $"remoteParticipants={room.RemoteParticipants.Count}");
            foreach (RemoteParticipant remoteParticipant in room.RemoteParticipants.Values)
            {
                Debug.Log(
                    $"[LiveKit] Participant present identity={remoteParticipant.Identity} " +
                    $"sid={remoteParticipant.Sid}");
            }
            subtitleHistoryView?.SetConnectionWaiting(false);
            connected?.Invoke();
            StartCoroutine(RequestSubtitleHistory());
            StartCoroutine(LoadSubtitleHistory(resolvedEndpoint, resolvedAccessCode));
        }

        // 접속 직후 기존 참가자에게 누적 자막 기록을 명시적으로 요청한다.
        private IEnumerator RequestSubtitleHistory()
        {
            if (!IsConnected || room.LocalParticipant == null)
            {
                yield break;
            }

            byte[] payload = CreateSubtitleSyncRequestPayload(
                DateTimeOffset.UtcNow.ToUnixTimeMilliseconds());
            PublishDataInstruction publish = room.LocalParticipant.PublishData(
                payload,
                destination_identities: null,
                reliable: true,
                topic: SubtitleSyncTopic);
            yield return publish;

            if (publish.IsError)
            {
                Debug.LogWarning($"[LiveKit] 자막 기록 요청 실패: {publish.Error}", this);
            }
        }

        // 외부 UI와 동일한 subtitle.sync.request 메시지를 생성한다.
        private static byte[] CreateSubtitleSyncRequestPayload(long timestamp)
        {
            SubtitleMessage request = new SubtitleMessage
            {
                type = "subtitle.sync.request",
                timestamp = timestamp,
            };
            return Encoding.UTF8.GetBytes(JsonUtility.ToJson(request));
        }

        // 상대 참가자의 응답을 놓친 경우 서버에 저장된 자막 기록을 보완 수신한다.
        private IEnumerator LoadSubtitleHistory(string resolvedTokenEndpoint, string resolvedAccessCode)
        {
            string historyEndpoint = CreateSubtitleHistoryEndpoint(resolvedTokenEndpoint);
            if (string.IsNullOrWhiteSpace(historyEndpoint))
            {
                yield break;
            }

            using UnityWebRequest request = UnityWebRequest.Get(historyEndpoint);
            request.SetRequestHeader("X-Access-Code", resolvedAccessCode ?? string.Empty);
            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                Debug.LogWarning($"[LiveKit] 자막 기록 불러오기 실패: {request.error}", this);
                yield break;
            }

            SubtitleHistoryResponse response = JsonUtility.FromJson<SubtitleHistoryResponse>(
                request.downloadHandler.text);
            if (response?.entries == null)
            {
                yield break;
            }

            foreach (SubtitleLogEntry entry in response.entries)
            {
                AppendSubtitleEntry(entry);
            }
        }

        // 토큰 API와 같은 서버의 자막 기록 API 주소를 생성한다.
        private static string CreateSubtitleHistoryEndpoint(string resolvedTokenEndpoint)
        {
            if (!Uri.TryCreate(resolvedTokenEndpoint, UriKind.Absolute, out Uri tokenUri))
            {
                return null;
            }

            UriBuilder builder = new UriBuilder(tokenUri)
            {
                Path = "/api/subtitles",
                Query = string.Empty,
                Fragment = string.Empty,
            };
            return builder.Uri.AbsoluteUri;
        }

        // 새 환경변수를 우선하고 값이 없을 때만 기존 이름을 사용한다.
        private static string GetEnvironmentVariable(string primaryName, string legacyName)
        {
            string value = Environment.GetEnvironmentVariable(primaryName);
            return string.IsNullOrWhiteSpace(value)
                ? Environment.GetEnvironmentVariable(legacyName)
                : value;
        }

        // Quest 마이크 송출 상태를 전환한다.
        public void ToggleMicrophone()
        {
            if (operationInProgress || !IsConnected)
            {
                return;
            }

            if (IsMicrophoneEnabled)
            {
                StopMicrophone();
            }
            else
            {
                StartCoroutine(StartMicrophoneRoutine());
            }
        }

        // 마이크 권한을 확인하고 오디오 트랙을 발행한다.
        private IEnumerator StartMicrophoneRoutine()
        {
            operationInProgress = true;
            if (!Application.HasUserAuthorization(UserAuthorization.Microphone))
            {
                yield return Application.RequestUserAuthorization(UserAuthorization.Microphone);
            }

            if (!Application.HasUserAuthorization(UserAuthorization.Microphone))
            {
                statusChanged?.Invoke("마이크 권한이 거부되었습니다.");
                operationInProgress = false;
                yield break;
            }

            string deviceName = SelectMicrophoneDevice();
            if (string.IsNullOrWhiteSpace(deviceName))
            {
                statusChanged?.Invoke("사용 가능한 마이크를 찾지 못했습니다.");
                operationInProgress = false;
                yield break;
            }

            microphoneObject = new GameObject("LiveKit Quest Microphone");
            microphoneObject.transform.SetParent(transform, false);
            microphoneSource = new MicrophoneSource(deviceName, microphoneObject);
            localAudioTrack = LocalAudioTrack.CreateAudioTrack(
                "quest-microphone",
                microphoneSource,
                room);

            TrackPublishOptions options = new TrackPublishOptions
            {
                AudioEncoding = new AudioEncoding { MaxBitrate = 64000 },
                Source = TrackSource.SourceMicrophone
            };
            PublishTrackInstruction publish = room.LocalParticipant.PublishTrack(
                localAudioTrack,
                options);
            yield return publish;
            operationInProgress = false;

            if (publish.IsError)
            {
                statusChanged?.Invoke("마이크 트랙 발행에 실패했습니다.");
                DisposeMicrophone();
                yield break;
            }

            microphoneSource.Start();
            statusChanged?.Invoke($"마이크 송출 중: {deviceName}");
            microphoneStateChanged?.Invoke(true);
        }

        // Quest Link 마이크를 우선해 입력 장치를 선택한다.
        private string SelectMicrophoneDevice()
        {
            string[] devices = Microphone.devices;
            if (devices == null || devices.Length == 0)
            {
                return null;
            }

            if (!string.IsNullOrWhiteSpace(preferredMicrophoneNameContains))
            {
                foreach (string device in devices)
                {
                    if (device.IndexOf(
                        preferredMicrophoneNameContains,
                        StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        return device;
                    }
                }
            }

            return devices[0];
        }

        // 발행 중인 Quest 마이크 트랙을 중지한다.
        public void StopMicrophone()
        {
            if (room != null && localAudioTrack != null)
            {
                room.LocalParticipant.UnpublishTrack(localAudioTrack, false);
            }

            DisposeMicrophone();
            statusChanged?.Invoke("마이크 송출 중지");
            microphoneStateChanged?.Invoke(false);
        }

        // 원격 마이크 트랙을 Unity AudioSource에 연결한다.
        private void OnTrackSubscribed(
            IRemoteTrack track,
            RemoteTrackPublication publication,
            RemoteParticipant participant)
        {
            if (track is RemoteVideoTrack videoTrack)
            {
                Debug.Log(
                    $"[LiveKit] Video track subscribed participant={participant?.Identity} " +
                    $"participantSid={participant?.Sid} track={publication?.Name} " +
                    $"trackSid={videoTrack.Sid} source={publication?.Source}");
                AddRemoteVideo(videoTrack, publication, participant);
                return;
            }

            if (track is not RemoteAudioTrack audioTrack)
            {
                return;
            }

            GameObject audioObject = new GameObject($"LiveKit Audio: {participant.Identity}");
            audioObject.transform.SetParent(transform, false);
            AudioSource source = audioObject.AddComponent<AudioSource>();
            if (remoteAudioTemplate != null)
            {
                source.spatialBlend = remoteAudioTemplate.spatialBlend;
                source.volume = remoteAudioTemplate.volume;
                source.outputAudioMixerGroup = remoteAudioTemplate.outputAudioMixerGroup;
            }
            else
            {
                source.spatialBlend = 0f;
            }

            audioObjects[audioTrack.Sid] = audioObject;
            audioStreams[audioTrack.Sid] = new AudioStream(audioTrack, source);
            statusChanged?.Invoke($"원격 음성 수신: {participant.Identity}");
        }

        // 구독 해제된 원격 오디오 리소스를 정리한다.
        private void OnTrackUnsubscribed(
            IRemoteTrack track,
            RemoteTrackPublication publication,
            RemoteParticipant participant)
        {
            if (track is RemoteVideoTrack videoTrack)
            {
                Debug.LogWarning(
                    $"[LiveKit] Video track unsubscribed participant={participant?.Identity} " +
                    $"participantSid={participant?.Sid} track={publication?.Name} " +
                    $"trackSid={videoTrack.Sid}");
                RemoveRemoteVideo(videoTrack.Sid);
            }
            else if (track is RemoteAudioTrack audioTrack)
            {
                RemoveRemoteAudio(audioTrack.Sid);
            }
        }

        // 지정한 Publisher의 카메라 영상을 Pilot 또는 정찰 드론 화면에 연결한다.
        private void AddRemoteVideo(
            RemoteVideoTrack videoTrack,
            RemoteTrackPublication publication,
            RemoteParticipant participant)
        {
            if (publication.Source != TrackSource.SourceCamera || participant == null)
            {
                Debug.Log(
                    $"[LiveKit] Video track ignored reason=non-camera-or-missing-participant " +
                    $"participant={participant?.Identity} track={publication?.Name} " +
                    $"trackSid={videoTrack.Sid} source={publication?.Source}");
                return;
            }

            LiveKitVideoSurface reconnaissanceSurface =
                reconnaissanceVideoSurface != null &&
                reconnaissanceVideoSurface.Matches(participant.Identity, publication.Name)
                    ? reconnaissanceVideoSurface
                    : null;
            bool targetsPilot = ShouldTargetPilotVideo(
                videoSourceSelector != null && videoSourceSelector.UsesLiveKit,
                videoPublisherIdentity,
                participant.Identity,
                reconnaissanceSurface != null);
            if (!targetsPilot && reconnaissanceSurface == null)
            {
                Debug.Log(
                    $"[LiveKit] Video track ignored reason=surface-filter " +
                    $"participant={participant.Identity} track={publication.Name} " +
                    $"trackSid={videoTrack.Sid} expectedReconParticipant=" +
                    $"{reconnaissanceVideoSurface?.PublisherIdentity} expectedReconTrack=" +
                    $"{reconnaissanceVideoSurface?.TrackName} pilotLiveKit=" +
                    $"{(videoSourceSelector != null && videoSourceSelector.UsesLiveKit)}");
                return;
            }

            if (RemoveConflictingVideoStreams(targetsPilot, reconnaissanceSurface))
            {
                statusChanged?.Invoke("원격 영상 트랙 교체");
            }

            string sid = videoTrack.Sid;
            VideoStream stream = new VideoStream(videoTrack);
            VideoStream.TextureReceiveDelegate handler = texture =>
            {
                if (targetsPilot)
                {
                    videoSourceSelector?.SetLiveKitTexture(texture);
                }
                reconnaissanceSurface?.SetTexture(texture);
                if (videoFirstFrames.Add(sid))
                {
                    statusChanged?.Invoke(
                        $"원격 영상 수신: {participant.Identity} · {publication.Name}");
                    Debug.Log(
                        $"[LiveKit] Video first frame participant={participant.Identity} " +
                        $"track={publication.Name} trackSid={sid} " +
                        $"size={texture?.width}x{texture?.height}");
                }
            };

            stream.TextureReceived += handler;
            stream.Start();
            videoBindings[sid] = new RemoteVideoBinding
            {
                Stream = stream,
                Coroutine = StartCoroutine(stream.Update()),
                Handler = handler,
                TargetsPilot = targetsPilot,
                ReconnaissanceSurface = reconnaissanceSurface,
                ParticipantIdentity = participant.Identity,
                TrackName = publication.Name
            };
            Debug.Log(
                $"[LiveKit] Video track bound participant={participant.Identity} " +
                $"track={publication.Name} trackSid={sid} " +
                $"target={(targetsPilot ? "pilot" : "recon")}");
        }

        // 정찰 트랙은 주 드론 화면과 겹치지 않도록 정찰 화면에만 연결한다.
        private static bool ShouldTargetPilotVideo(
            bool usesPilotLiveKit,
            string pilotPublisherIdentity,
            string participantIdentity,
            bool targetsReconnaissance)
        {
            return usesPilotLiveKit &&
                   !targetsReconnaissance &&
                   (string.IsNullOrWhiteSpace(pilotPublisherIdentity) ||
                    string.Equals(
                        participantIdentity,
                        pilotPublisherIdentity,
                        StringComparison.Ordinal));
        }

        // 같은 출력 화면을 사용하는 이전 영상 스트림만 교체한다.
        private bool RemoveConflictingVideoStreams(
            bool targetsPilot,
            LiveKitVideoSurface reconnaissanceSurface)
        {
            List<string> conflictingStreamIds = new List<string>();
            foreach (KeyValuePair<string, RemoteVideoBinding> item in videoBindings)
            {
                RemoteVideoBinding binding = item.Value;
                if ((targetsPilot && binding.TargetsPilot) ||
                    (reconnaissanceSurface != null &&
                     binding.ReconnaissanceSurface == reconnaissanceSurface))
                {
                    conflictingStreamIds.Add(item.Key);
                }
            }

            foreach (string streamId in conflictingStreamIds)
            {
                RemoveRemoteVideo(streamId);
            }

            return conflictingStreamIds.Count > 0;
        }

        // 구독 해제된 원격 비디오 스트림을 정리한다.
        private void RemoveRemoteVideo(string sid)
        {
            if (videoBindings.Remove(sid, out RemoteVideoBinding binding))
            {
                Debug.LogWarning(
                    $"[LiveKit] Video track removed participant={binding.ParticipantIdentity} " +
                    $"track={binding.TrackName} trackSid={sid} " +
                    $"target={(binding.TargetsPilot ? "pilot" : "recon")}");
                binding.Stream.TextureReceived -= binding.Handler;
                binding.Stream.Stop();
                if (binding.Coroutine != null)
                {
                    StopCoroutine(binding.Coroutine);
                }
                binding.Stream.Dispose();
                if (binding.TargetsPilot)
                {
                    videoSourceSelector?.ClearLiveKitTexture();
                }
                binding.ReconnaissanceSurface?.ClearTexture();
            }

            videoFirstFrames.Remove(sid);
            if (videoBindings.Count == 0)
            {
                statusChanged?.Invoke("원격 영상 수신 중지");
            }
        }

        // 모든 원격 비디오 스트림을 정리한다.
        private void DisposeVideoStreams()
        {
            if (videoBindings.Count == 0)
            {
                videoSourceSelector?.ClearLiveKitTexture();
                reconnaissanceVideoSurface?.ClearTexture();
                return;
            }

            string[] streamIds = new string[videoBindings.Count];
            videoBindings.Keys.CopyTo(streamIds, 0);
            foreach (string sid in streamIds)
            {
                RemoveRemoteVideo(sid);
            }
        }

        // LiveKit 참가자가 발행한 자막과 동기화 기록을 그대로 전달한다.
        private void OnDataReceived(
            byte[] data,
            Participant participant,
            DataPacketKind kind,
            string topic)
        {
            if (!ShouldAcceptSubtitlePacket(topic, participant?.Identity))
            {
                return;
            }

            SubtitleMessage message = JsonUtility.FromJson<SubtitleMessage>(
                Encoding.UTF8.GetString(data));
            if (topic == SubtitleSyncTopic)
            {
                if (message?.type != "subtitle.sync.state" || message.entries == null)
                {
                    return;
                }

                foreach (SubtitleLogEntry entry in message.entries)
                {
                    AppendSubtitleEntry(entry);
                }
                return;
            }

            if (message?.type == "subtitle.history.clear")
            {
                if (participant.Identity.StartsWith("admin-", StringComparison.Ordinal))
                {
                    subtitleEntryIds.Clear();
                    subtitleHistoryView?.Clear();
                }
                return;
            }

            if (TryFormatSubtitle(message, out string formattedSubtitle))
            {
                string entryId = message.type == "subtitle.log"
                    ? message.entry?.id
                    : message.id;
                AppendSubtitle(entryId, formattedSubtitle);
            }
        }

        // 외부 UI 계약에 정의된 자막 topic과 유효 참가자만 수신한다.
        private static bool ShouldAcceptSubtitlePacket(string topic, string participantIdentity)
        {
            return !string.IsNullOrWhiteSpace(participantIdentity) &&
                (topic == SubtitleTopic || topic == SubtitleSyncTopic);
        }

        // 동기화 기록 한 건을 표시 형식으로 변환해 추가한다.
        private void AppendSubtitleEntry(SubtitleLogEntry entry)
        {
            if (entry == null)
            {
                return;
            }

            SubtitleMessage message = new SubtitleMessage
            {
                type = "subtitle.log",
                entry = entry,
            };
            if (TryFormatSubtitle(message, out string formattedSubtitle))
            {
                AppendSubtitle(entry.id, formattedSubtitle);
            }
        }

        // 실시간 전송과 기록 동기화로 같은 자막이 중복 표시되지 않게 한다.
        private void AppendSubtitle(string entryId, string formattedSubtitle)
        {
            if (!string.IsNullOrWhiteSpace(entryId) && !subtitleEntryIds.Add(entryId))
            {
                return;
            }

            subtitleHistoryView?.Append(formattedSubtitle);
            subtitleReceived?.Invoke(formattedSubtitle);
        }

        // 현재·신규 자막 메시지 형식을 동일한 표시 문자열로 변환한다.
        private static bool TryFormatSubtitle(
            SubtitleMessage message,
            out string formattedSubtitle)
        {
            formattedSubtitle = null;
            if (message == null)
            {
                return false;
            }

            string speaker;
            string text;
            if (message.type == "subtitle")
            {
                if (!message.isFinal)
                {
                    return false;
                }

                speaker = message.speakerLabel;
                text = message.text;
            }
            else if (message.type == "subtitle.log" && message.entry != null)
            {
                speaker = message.entry.speaker;
                text = message.entry.text;
            }
            else
            {
                return false;
            }

            if (string.IsNullOrWhiteSpace(text))
            {
                return false;
            }

            formattedSubtitle =
                $"{(string.IsNullOrWhiteSpace(speaker) ? "관리자" : speaker)}: {text}";
            return true;
        }

        // 참가자 입장 시 영상 Publisher 식별에 필요한 상태를 기록한다.
        private void OnParticipantConnected(Participant participant)
        {
            Debug.Log(
                $"[LiveKit] Participant joined identity={participant?.Identity} " +
                $"sid={participant?.Sid}");
        }

        // 참가자 퇴장 사유를 영상 구독 해제와 함께 대조할 수 있도록 기록한다.
        private void OnParticipantDisconnected(
            Participant participant,
            DisconnectReason reason)
        {
            Debug.LogWarning(
                $"[LiveKit] Participant left identity={participant?.Identity} " +
                $"sid={participant?.Sid} reason={reason}");
        }

        // LiveKit 연결 종료 상태를 반영한다.
        private void OnDisconnected(Room disconnectedRoom)
        {
            Debug.LogWarning(
                $"[LiveKit] Room disconnected reason={disconnectedRoom?.DisconnectReason} " +
                $"requested={disconnectRequested}");
            DisposeAudioStreams();
            DisposeVideoStreams();
            DisposeMicrophone();
            operationInProgress = false;
            subtitleHistoryView?.SetConnectionWaiting(true);
            statusChanged?.Invoke("LiveKit 연결 종료");
            disconnected?.Invoke();

            ScheduleReconnect();
        }

        // 최초 연결과 연결 종료 실패를 동일한 재접속 경로로 복구한다.
        private void ScheduleReconnect()
        {
            if (disconnectRequested || !connectOnEnable || !isActiveAndEnabled ||
                reconnectCoroutine != null)
            {
                return;
            }

            reconnectCoroutine = StartCoroutine(ReconnectRoutine());
        }

        // 최종 연결 종료 후 Room을 정리하고 다시 접속한다.
        private IEnumerator ReconnectRoutine()
        {
            float delaySeconds = CalculateReconnectDelaySeconds(reconnectAttempt);
            Debug.LogWarning(
                $"[LiveKit] Reconnect scheduled attempt={reconnectAttempt + 1} " +
                $"delaySeconds={delaySeconds:0.#}");
            reconnectAttempt++;
            yield return new WaitForSecondsRealtime(delaySeconds);
            reconnectCoroutine = null;
            if (disconnectRequested || !isActiveAndEnabled)
            {
                yield break;
            }

            DisposeRoom();
            Connect();
        }

        // 반복 실패 시 서버 요청 간격을 늘리되 복구 지연은 30초로 제한한다.
        private static float CalculateReconnectDelaySeconds(int attempt)
        {
            int clampedAttempt = Mathf.Clamp(attempt, 0, 4);
            return Mathf.Min(
                ReconnectBaseDelaySeconds * Mathf.Pow(2f, clampedAttempt),
                ReconnectMaxDelaySeconds);
        }

        // LiveKit Room 연결을 종료한다.
        public void Disconnect()
        {
            disconnectRequested = true;
            Debug.Log("[LiveKit] Room disconnect requested");
            reconnectAttempt = 0;
            if (reconnectCoroutine != null)
            {
                StopCoroutine(reconnectCoroutine);
                reconnectCoroutine = null;
            }
            StopMicrophone();
            subtitleHistoryView?.SetConnectionWaiting(true);
            room?.Disconnect();
            DisposeRoom();
        }

        // 원격 오디오 하나를 정리한다.
        private void RemoveRemoteAudio(string sid)
        {
            if (audioStreams.Remove(sid, out AudioStream stream))
            {
                stream.Dispose();
            }

            if (audioObjects.Remove(sid, out GameObject audioObject) &&
                audioObject != null)
            {
                Destroy(audioObject);
            }
        }

        // 모든 원격 오디오 리소스를 정리한다.
        private void DisposeAudioStreams()
        {
            foreach (AudioStream stream in audioStreams.Values)
            {
                stream.Dispose();
            }
            audioStreams.Clear();

            foreach (GameObject audioObject in audioObjects.Values)
            {
                if (audioObject != null)
                {
                    Destroy(audioObject);
                }
            }
            audioObjects.Clear();
        }

        // 로컬 마이크 리소스를 정리한다.
        private void DisposeMicrophone()
        {
            microphoneSource?.Stop();
            microphoneSource?.Dispose();
            microphoneSource = null;
            localAudioTrack = null;
            if (microphoneObject != null)
            {
                Destroy(microphoneObject);
                microphoneObject = null;
            }
        }

        // Room 이벤트와 네이티브 리소스를 정리한다.
        private void DisposeRoom()
        {
            if (room == null)
            {
                return;
            }

            room.TrackSubscribed -= OnTrackSubscribed;
            room.TrackUnsubscribed -= OnTrackUnsubscribed;
            room.ParticipantConnected -= OnParticipantConnected;
            room.ParticipantDisconnectedWithReason -= OnParticipantDisconnected;
            room.DataReceived -= OnDataReceived;
            room.Disconnected -= OnDisconnected;
            room.Dispose();
            room = null;
        }

        // 오브젝트 종료 시 LiveKit 리소스를 정리한다.
        private void OnDestroy()
        {
            disconnectRequested = true;
            if (reconnectCoroutine != null)
            {
                StopCoroutine(reconnectCoroutine);
                reconnectCoroutine = null;
            }
            DisposeAudioStreams();
            DisposeVideoStreams();
            DisposeMicrophone();
            room?.Disconnect();
            DisposeRoom();
        }
    }
}
