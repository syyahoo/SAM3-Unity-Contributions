using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using StrikerPC.Video;
using UnityEngine;

namespace StrikerPC.Map
{
    public sealed class MapPingUdpBridge : MonoBehaviour
    {
        private const int MaximumAnnotationSyncAttempts = 30;
        private const float AnnotationSyncRetrySeconds = 1f;
        private const float PingSyncRetrySeconds = 1f;

        [SerializeField] private WorldPingManager pingManager;
        [SerializeField] private PilotAnnotationOverlay annotationOverlay;
        [SerializeField] private MissionVideoOverlay missionOverlay;
        [SerializeField, Min(1)] private int commandPort = 49300;
        [SerializeField, Min(1)] private int eventPort = 49301;

        private readonly ConcurrentQueue<string> commands = new ConcurrentQueue<string>();
        private UdpClient client;
        private Thread receiverThread;
        private volatile bool receiverRunning;
        private string pendingError;
        private int annotationSyncAttempts;
        private bool missionSyncRequested;
        private float nextAnnotationSyncTime;
        private float nextPingSyncTime;
        private string activeSnapshotId;
        private int expectedSnapshotChunks;
        private readonly Dictionary<int, WorldPingEvent[]> snapshotChunks =
            new Dictionary<int, WorldPingEvent[]>();

        public bool IsMapVerified { get; private set; }
        public bool HasReceivedPingSnapshot { get; private set; }
        public event Action<string, string> CommandRejected;

        // Gateway의 localhost 명령 수신을 시작한다.
        private void OnEnable()
        {
            try
            {
                client = new UdpClient(new IPEndPoint(IPAddress.Loopback, commandPort));
                client.Client.ReceiveTimeout = 500;
                receiverRunning = true;
                receiverThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "MapPingGatewayReceiver"
                };
                receiverThread.Start();
                RequestAnnotationSync();
                RequestPingSync();
                RequestMissionSync();
            }
            catch (Exception exception)
            {
                Debug.LogError($"[Map Ping] UDP 수신을 시작하지 못했습니다: {exception.Message}", this);
            }
        }

        // 수신 명령을 Unity 메인 스레드에서 처리한다.
        private void Update()
        {
            int processed = 0;
            while (processed < 16 && commands.TryDequeue(out string json))
            {
                ProcessCommand(json);
                processed++;
            }

            if (annotationOverlay != null
                && !annotationOverlay.HasReceivedSnapshot
                && annotationSyncAttempts < MaximumAnnotationSyncAttempts
                && Time.unscaledTime >= nextAnnotationSyncTime)
            {
                RequestAnnotationSync();
            }

            if ((!IsMapVerified || !HasReceivedPingSnapshot)
                && Time.unscaledTime >= nextPingSyncTime)
            {
                RequestPingSync();
            }

            if (!string.IsNullOrEmpty(pendingError))
            {
                string message = pendingError;
                pendingError = null;
                Debug.LogWarning($"[Map Ping] {message}", this);
            }
        }

        // UDP 수신 스레드와 소켓을 종료한다.
        private void OnDisable()
        {
            receiverRunning = false;
            client?.Close();
            if (receiverThread != null && receiverThread.IsAlive)
            {
                receiverThread.Join(1000);
            }

            receiverThread = null;
            client = null;
            annotationSyncAttempts = 0;
            missionSyncRequested = false;
            nextAnnotationSyncTime = 0f;
            nextPingSyncTime = 0f;
            IsMapVerified = false;
            HasReceivedPingSnapshot = false;
            activeSnapshotId = null;
            expectedSnapshotChunks = 0;
            snapshotChunks.Clear();
            while (commands.TryDequeue(out _))
            {
            }
        }

        // localhost UDP 명령을 제한 큐에 넣는다.
        private void ReceiveLoop()
        {
            IPEndPoint sender = null;
            while (receiverRunning)
            {
                try
                {
                    byte[] payload = client.Receive(ref sender);
                    while (commands.Count >= 256 && commands.TryDequeue(out _))
                    {
                    }
                    commands.Enqueue(Encoding.UTF8.GetString(payload));
                }
                catch (SocketException exception) when (exception.SocketErrorCode == SocketError.TimedOut)
                {
                }
                catch (Exception exception)
                {
                    if (receiverRunning)
                    {
                        pendingError = exception.Message;
                    }
                }
            }
        }

        // 명령 종류에 따라 생성·삭제·초기화를 수행한다.
        private void ProcessCommand(string json)
        {
            MapCommandHeader header = JsonUtility.FromJson<MapCommandHeader>(json);
            if (header == null || string.IsNullOrWhiteSpace(header.type))
            {
                return;
            }

            if (header.type == "annotation-event"
                || header.type == "annotation-delete"
                || header.type == "annotation-clear"
                || header.type == "annotation-snapshot")
            {
                annotationOverlay?.ProcessCommand(json);
            }
            else if (header.type == "mission-detection-frame"
                || header.type == "mission-target-state")
            {
                missionOverlay?.ProcessCommand(json);
            }
            else if (pingManager == null)
            {
                return;
            }
            else if (header.type == "map-ping-request")
            {
                ProcessCreate(JsonUtility.FromJson<MapPingRequest>(json));
            }
            else if (header.type == "map-state")
            {
                ProcessMapState(JsonUtility.FromJson<MapStateMessage>(json));
            }
            else if (header.type == "ping-snapshot")
            {
                ProcessPingSnapshot(JsonUtility.FromJson<PingSnapshotMessage>(json));
            }
            else if (header.type == "ping-event")
            {
                WorldPingEvent pingEvent = JsonUtility.FromJson<WorldPingEvent>(json);
                if (!pingManager.TryRestorePing(pingEvent, out string reason))
                {
                    pendingError = reason;
                }
            }
            else if (header.type == "ping-rejected")
            {
                PingRejectedEvent rejected = JsonUtility.FromJson<PingRejectedEvent>(json);
                CommandRejected?.Invoke(
                    rejected?.requestId,
                    rejected?.reason ?? "핑 요청이 거부되었습니다.");
            }
            else if (header.type == "ping-delete")
            {
                if (pingManager.RemovePing(header.id))
                {
                    SendEvent(new PingDeletedEvent
                    {
                        id = header.id,
                        requestId = header.requestId
                    });
                }
                else
                {
                    SendEvent(new PingRejectedEvent
                    {
                        requestId = header.requestId,
                        reason = "삭제할 핑이 없습니다."
                    });
                }
            }
            else if (header.type == "ping-clear")
            {
                pingManager.ClearAll();
                SendRaw("{\"type\":\"ping-cleared\"}");
            }
        }

        // Gateway에 현재 영상 주석 상태 동기화를 요청한다.
        private void RequestAnnotationSync()
        {
            if (client == null || annotationOverlay == null)
            {
                return;
            }

            SendRaw("{\"type\":\"annotation-sync-request\"}");
            annotationSyncAttempts++;
            nextAnnotationSyncTime = Time.unscaledTime + AnnotationSyncRetrySeconds;
        }

        // Gateway에 지도 검증 정보와 현재 핑 스냅샷을 요청한다.
        private void RequestPingSync()
        {
            if (client == null)
            {
                return;
            }

            SendRaw("{\"type\":\"ping-sync-request\"}");
            nextPingSyncTime = Time.unscaledTime + PingSyncRetrySeconds;
        }

        // Gateway에 유지 중인 임무 목표 상태 동기화를 요청한다.
        private void RequestMissionSync()
        {
            if (client == null || missionSyncRequested)
            {
                return;
            }

            SendRaw("{\"type\":\"mission-sync-request\"}");
            missionSyncRequested = true;
        }

        // Scene 직렬화 변경 없이 임무 영상 오버레이를 연결한다.
        public void ConfigureMissionOverlay(MissionVideoOverlay overlay)
        {
            missionOverlay = overlay;
            if (isActiveAndEnabled)
            {
                RequestMissionSync();
            }
        }

        // 지도 핑을 지면에 확정하고 결과를 Gateway로 보낸다.
        private void ProcessCreate(MapPingRequest request)
        {
            string reason = "잘못된 핑 요청입니다.";
            if (request != null && pingManager.TryCreatePing(request, out WorldPingEvent pingEvent, out reason))
            {
                SendEvent(pingEvent);
                return;
            }

            SendEvent(new PingRejectedEvent
            {
                requestId = request?.requestId,
                requesterIdentity = request?.requesterIdentity,
                requesterRole = request?.requesterRole,
                reason = reason
            });
        }

        // Gateway 지도 manifest와 로컬 StreamingAssets 파일이 동일한지 확인한다.
        private void ProcessMapState(MapStateMessage state)
        {
            IsMapVerified = false;
            VenueMapDefinition map = pingManager?.Map;
            if (state == null
                || map == null
                || state.coordinateSpace != MapCoordinateConverter.VenueXzMeters
                || state.mapId != map.mapId
                || state.mapVersion != map.version
                || state.scanId != map.scanId
                || !Mathf.Approximately(state.minX, map.minX)
                || !Mathf.Approximately(state.maxX, map.maxX)
                || !Mathf.Approximately(state.minZ, map.minZ)
                || !Mathf.Approximately(state.maxZ, map.maxZ))
            {
                pingManager?.ClearAll();
                CommandRejected?.Invoke(
                    null,
                    "Gateway 지도 ID, 버전 또는 경계가 로컬 지도와 일치하지 않습니다.");
                return;
            }

            string mapDirectory = Path.Combine(Application.streamingAssetsPath, "Maps");
            string manifestPath = Path.Combine(mapDirectory, "venue-map.json");
            string imagePath = Path.Combine(
                mapDirectory,
                Path.GetFileName(map.imagePath));
            try
            {
                string manifestHash = ComputeSha256(manifestPath);
                string imageHash = ComputeSha256(imagePath);
                IsMapVerified = string.Equals(
                        manifestHash,
                        state.manifestSha256,
                        StringComparison.OrdinalIgnoreCase)
                    && string.Equals(
                        imageHash,
                        state.imageSha256,
                        StringComparison.OrdinalIgnoreCase);
            }
            catch (Exception exception)
            {
                pendingError = exception.Message;
            }

            if (!IsMapVerified)
            {
                pingManager.ClearAll();
                CommandRejected?.Invoke(
                    null,
                    "Gateway 지도 해시가 로컬 지도와 일치하지 않습니다.");
            }
        }

        // 분할된 핑 스냅샷을 모두 받은 뒤 한 번에 현재 상태로 교체한다.
        private void ProcessPingSnapshot(PingSnapshotMessage snapshot)
        {
            if (snapshot == null
                || string.IsNullOrWhiteSpace(snapshot.snapshotId)
                || snapshot.chunkCount < 1
                || snapshot.chunkIndex < 0
                || snapshot.chunkIndex >= snapshot.chunkCount)
            {
                return;
            }

            if (activeSnapshotId != snapshot.snapshotId)
            {
                activeSnapshotId = snapshot.snapshotId;
                expectedSnapshotChunks = snapshot.chunkCount;
                snapshotChunks.Clear();
            }
            if (snapshot.chunkCount != expectedSnapshotChunks)
            {
                return;
            }

            snapshotChunks[snapshot.chunkIndex] = snapshot.pings ?? Array.Empty<WorldPingEvent>();
            if (snapshotChunks.Count != expectedSnapshotChunks || !IsMapVerified)
            {
                return;
            }

            List<WorldPingEvent> combined = new List<WorldPingEvent>();
            for (int index = 0; index < expectedSnapshotChunks; index++)
            {
                if (!snapshotChunks.TryGetValue(index, out WorldPingEvent[] items))
                {
                    return;
                }
                combined.AddRange(items);
            }

            pingManager.ReplaceFromSnapshot(combined);
            HasReceivedPingSnapshot = true;
            snapshotChunks.Clear();
        }

        // Quest 지도 클릭을 웹과 같은 map-ping-request 계약으로 처리한다.
        public bool TryCreateLocalPing(
            float mapX,
            float mapZ,
            string requesterIdentity,
            out string requestId)
        {
            requestId = null;
            VenueMapDefinition map = pingManager?.Map;
            if (map == null || !IsMapVerified)
            {
                return false;
            }

            requestId = Guid.NewGuid().ToString("N");
            MapPingRequest request = new MapPingRequest
            {
                type = "map-ping-request",
                requestId = requestId,
                mapId = map.mapId,
                mapVersion = map.version,
                scanId = map.scanId,
                mapX = mapX,
                mapZ = mapZ,
                requesterIdentity = requesterIdentity,
                requesterRole = "pilot",
                timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
            };
            SendEvent(request);
            return true;
        }

        // 파일럿이 소유한 핑의 삭제를 Gateway에 요청한다.
        public bool TryDeleteLocalPing(string id, string requesterIdentity)
        {
            if (!IsMapVerified
                || pingManager == null
                || !pingManager.IsOwnedByLocalPilot(id))
            {
                return false;
            }

            SendRaw(JsonUtility.ToJson(new LocalPingDeleteRequest
            {
                requestId = Guid.NewGuid().ToString("N"),
                id = id,
                requesterIdentity = requesterIdentity
            }));
            return true;
        }

        // 파일의 SHA-256을 소문자 16진수로 반환한다.
        private static string ComputeSha256(string path)
        {
            using SHA256 sha = SHA256.Create();
            using FileStream stream = File.OpenRead(path);
            byte[] hash = sha.ComputeHash(stream);
            StringBuilder builder = new StringBuilder(hash.Length * 2);
            foreach (byte value in hash)
            {
                builder.Append(value.ToString("x2"));
            }
            return builder.ToString();
        }

        // 직렬화 가능한 이벤트를 Gateway로 보낸다.
        private void SendEvent(object message)
        {
            SendRaw(JsonUtility.ToJson(message));
        }

        // UTF-8 JSON 이벤트를 localhost Gateway로 보낸다.
        private void SendRaw(string json)
        {
            try
            {
                byte[] payload = Encoding.UTF8.GetBytes(json);
                client.Send(payload, payload.Length, new IPEndPoint(IPAddress.Loopback, eventPort));
            }
            catch (Exception exception)
            {
                pendingError = exception.Message;
            }
        }

        [Serializable]
        private sealed class LocalPingDeleteRequest
        {
            public string type = "ping-delete-request";
            public string requestId;
            public string id;
            public string requesterIdentity;
        }
    }
}
