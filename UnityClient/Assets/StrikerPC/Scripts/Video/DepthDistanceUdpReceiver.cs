using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace StrikerPC.Video
{
    [DisallowMultipleComponent]
    public sealed class DepthDistanceUdpReceiver : MonoBehaviour
    {
        public const int DefaultUdpPort = 49005;

        [SerializeField] private int udpPort = DefaultUdpPort;

        private readonly ConcurrentQueue<string> pendingMessages =
            new ConcurrentQueue<string>();
        private DepthDistanceSample latestSample;
        private float latestReceivedAt = -1f;
        private long lastSequence;
        private ulong lastSourceTimestampNs;
        private UdpClient receiver;
        private Thread receiveThread;
        private volatile bool running;
        private string pendingSocketWarning;

        public bool HasSample => latestReceivedAt >= 0f;
        public DepthDistanceSample LatestSample => latestSample;

        // 플레이 모드에서만 localhost 거리 표본 수신을 시작한다.
        private void OnEnable()
        {
            if (Application.isPlaying)
            {
                StartReceiver();
            }
        }

        // 수신 스레드가 넘긴 JSON을 Unity 메인 스레드에서 검증한다.
        private void Update()
        {
            string warning = Interlocked.Exchange(ref pendingSocketWarning, null);
            if (!string.IsNullOrEmpty(warning))
            {
                Debug.LogWarning($"[Depth Distance] UDP 수신 오류: {warning}", this);
            }

            int processed = 0;
            while (processed < 16 && pendingMessages.TryDequeue(out string json))
            {
                TryAcceptMessage(json, Time.unscaledTime);
                processed++;
            }
        }

        // 테스트와 UDP 공통 경로로 순서가 맞는 최신 거리 표본만 수락한다.
        public bool TryAcceptMessage(string json, float receivedAt)
        {
            if (!DepthDistanceProtocol.TryParse(json, out DepthDistanceSample sample)
                || !IsFinite(receivedAt)
                || receivedAt < 0f)
            {
                return false;
            }

            bool newSession = lastSequence > 0
                && sample.Sequence <= 4
                && sample.Sequence <= lastSequence
                && sample.SourceTimestampNs > lastSourceTimestampNs + 250_000_000UL;
            if (sample.Sequence <= lastSequence && !newSession)
            {
                return false;
            }

            lastSequence = sample.Sequence;
            lastSourceTimestampNs = sample.SourceTimestampNs;
            latestSample = sample;
            latestReceivedAt = receivedAt;
            return true;
        }

        // 수신 age와 현재 FPV source timestamp가 모두 허용 범위인 표본만 반환한다.
        public bool TryGetFreshSample(
            float now,
            float maximumAgeSeconds,
            ulong frameSourceTimestampNs,
            ulong maximumTimestampDeltaNs,
            out DepthDistanceSample sample)
        {
            sample = default;
            if (!HasSample
                || frameSourceTimestampNs == 0UL
                || !IsFinite(now)
                || now < latestReceivedAt
                || now - latestReceivedAt > Mathf.Max(0f, maximumAgeSeconds))
            {
                return false;
            }

            ulong delta = latestSample.SourceTimestampNs > frameSourceTimestampNs
                ? latestSample.SourceTimestampNs - frameSourceTimestampNs
                : frameSourceTimestampNs - latestSample.SourceTimestampNs;
            if (delta > maximumTimestampDeltaNs)
            {
                return false;
            }

            sample = latestSample;
            return true;
        }

        // localhost UDP 소켓과 전용 수신 스레드를 시작한다.
        private void StartReceiver()
        {
            if (running)
            {
                return;
            }

            try
            {
                receiver = new UdpClient(new IPEndPoint(IPAddress.Loopback, udpPort));
                receiver.Client.ReceiveTimeout = 250;
                running = true;
                receiveThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "StrikerPC Depth Distance Receiver"
                };
                receiveThread.Start();
            }
            catch (Exception exception)
            {
                running = false;
                receiver?.Close();
                receiver = null;
                Interlocked.Exchange(ref pendingSocketWarning, exception.Message);
            }
        }

        // 작은 UDP payload만 제한된 큐에 넣고 Unity API 호출은 메인 스레드에 맡긴다.
        private void ReceiveLoop()
        {
            IPEndPoint sender = new IPEndPoint(IPAddress.Loopback, 0);
            while (running)
            {
                try
                {
                    byte[] payload = receiver.Receive(ref sender);
                    if (!IPAddress.IsLoopback(sender.Address)
                        || payload.Length == 0
                        || payload.Length > 2048)
                    {
                        continue;
                    }
                    while (pendingMessages.Count >= 64)
                    {
                        pendingMessages.TryDequeue(out _);
                    }
                    pendingMessages.Enqueue(Encoding.UTF8.GetString(payload));
                }
                catch (SocketException exception)
                {
                    if (running && exception.SocketErrorCode != SocketError.TimedOut)
                    {
                        Interlocked.Exchange(ref pendingSocketWarning, exception.Message);
                    }
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
            }
        }

        // 수신 스레드와 소켓을 안전하게 종료한다.
        private void OnDisable()
        {
            running = false;
            receiver?.Close();
            if (receiveThread != null && receiveThread.IsAlive)
            {
                receiveThread.Join(300);
            }
            receiveThread = null;
            receiver = null;
        }

        // float 값이 NaN 또는 Infinity가 아닌지 확인한다.
        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
