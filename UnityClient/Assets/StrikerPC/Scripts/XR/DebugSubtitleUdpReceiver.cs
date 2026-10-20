using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    public sealed class DebugSubtitleUdpReceiver : MonoBehaviour
    {
        [Serializable]
        private sealed class DebugSubtitlePayload
        {
            public int version;
            public string role;
            public string text;
            public bool clear;
            public bool visible;
        }

        private const int ProtocolVersion = 1;
        private const int MaximumDatagramBytes = 4096;
        private const int MaximumPendingMessages = 32;
        private const int MaximumTextCharacters = 500;
        private const int MaximumRoleCharacters = 24;

        [SerializeField, Min(1)] private int listenPort = 49303;
        [SerializeField] private SubtitleHistoryView subtitleHistoryView;

        private readonly ConcurrentQueue<string> pendingPayloads =
            new ConcurrentQueue<string>();
        private UdpClient receiver;
        private Thread receiveThread;
        private XrMovableWindow subtitleWindow;
        private volatile bool receiving;
        private volatile string pendingWarning;

        public bool IsListening => receiver != null && receiving;

        // 기존 자막 기록과 창을 디버그 입력 대상으로 연결한다.
        public void Configure(SubtitleHistoryView historyView)
        {
            subtitleHistoryView = historyView;
            subtitleWindow = historyView != null
                ? historyView.GetComponentInParent<XrMovableWindow>(true)
                : null;
        }

        // localhost 디버그 자막 수신을 시작한다.
        private void OnEnable()
        {
            StartReceiver();
        }

        // 수신 스레드가 받은 문자열만 Unity 메인 스레드에서 UI에 적용한다.
        private void Update()
        {
            if (!string.IsNullOrEmpty(pendingWarning))
            {
                string warning = pendingWarning;
                pendingWarning = null;
                Debug.LogWarning($"[Debug Subtitle] {warning}", this);
            }

            int processed = 0;
            while (processed < MaximumPendingMessages
                && pendingPayloads.TryDequeue(out string payload))
            {
                ApplyPayload(payload);
                processed++;
            }
        }

        // localhost 소켓과 수신 스레드를 정리한다.
        private void OnDisable()
        {
            receiving = false;
            receiver?.Close();
            receiver = null;
            if (receiveThread != null && receiveThread.IsAlive)
            {
                receiveThread.Join(500);
            }
            receiveThread = null;
            while (pendingPayloads.TryDequeue(out _))
            {
            }
        }

        // 역할과 본문을 길이 제한이 있는 디버그 자막 한 줄로 만든다.
        public static bool TryFormatSubtitle(
            string role,
            string text,
            out string subtitle)
        {
            subtitle = null;
            if (string.IsNullOrWhiteSpace(text))
            {
                return false;
            }

            string normalizedRole = string.IsNullOrWhiteSpace(role)
                ? "Codex"
                : role.Trim();
            if (normalizedRole.Length > MaximumRoleCharacters)
            {
                normalizedRole = normalizedRole.Substring(0, MaximumRoleCharacters);
            }

            string normalizedText = text.Trim();
            if (normalizedText.Length > MaximumTextCharacters)
            {
                normalizedText = normalizedText.Substring(0, MaximumTextCharacters);
            }

            subtitle = $"[DEBUG][{normalizedRole}] {normalizedText}";
            return true;
        }

        // 수신 소켓은 외부 LAN에서 접근할 수 없도록 IPv4 loopback에만 바인딩한다.
        private void StartReceiver()
        {
            if (receiving)
            {
                return;
            }

            try
            {
                receiver = new UdpClient(new IPEndPoint(IPAddress.Loopback, listenPort));
                receiving = true;
                receiveThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "StrikerPC-DebugSubtitle"
                };
                receiveThread.Start();
            }
            catch (Exception exception)
            {
                receiving = false;
                receiver?.Close();
                receiver = null;
                Debug.LogWarning(
                    $"[Debug Subtitle] UDP {listenPort} 수신 실패: {exception.Message}",
                    this);
            }
        }

        // 제한 크기의 localhost UDP 메시지만 메인 스레드 큐로 전달한다.
        private void ReceiveLoop()
        {
            IPEndPoint sender = new IPEndPoint(IPAddress.Loopback, 0);
            while (receiving)
            {
                try
                {
                    byte[] bytes = receiver.Receive(ref sender);
                    if (!IPAddress.IsLoopback(sender.Address)
                        || bytes.Length == 0
                        || bytes.Length > MaximumDatagramBytes)
                    {
                        continue;
                    }

                    while (pendingPayloads.Count >= MaximumPendingMessages
                        && pendingPayloads.TryDequeue(out _))
                    {
                    }
                    pendingPayloads.Enqueue(Encoding.UTF8.GetString(bytes));
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (SocketException exception)
                {
                    if (receiving)
                    {
                        pendingWarning = exception.Message;
                    }
                    break;
                }
            }
        }

        // 검증된 JSON 명령을 기존 자막 기록과 자막창 표시 상태에 반영한다.
        private void ApplyPayload(string json)
        {
            DebugSubtitlePayload payload;
            try
            {
                payload = JsonUtility.FromJson<DebugSubtitlePayload>(json);
            }
            catch (ArgumentException)
            {
                return;
            }

            if (payload == null || payload.version != ProtocolVersion)
            {
                return;
            }

            if (payload.clear)
            {
                subtitleHistoryView?.Clear();
            }

            SetSubtitleWindowVisible(payload.visible);
            if (TryFormatSubtitle(payload.role, payload.text, out string subtitle))
            {
                subtitleHistoryView?.Append(subtitle);
            }
        }

        // 디버그 중에만 기존 자막창을 표시하고 hide 명령으로 다시 숨긴다.
        private void SetSubtitleWindowVisible(bool visible)
        {
            if (subtitleWindow != null)
            {
                subtitleWindow.gameObject.SetActive(visible);
            }
            else if (subtitleHistoryView != null)
            {
                subtitleHistoryView.gameObject.SetActive(visible);
            }
        }
    }
}
