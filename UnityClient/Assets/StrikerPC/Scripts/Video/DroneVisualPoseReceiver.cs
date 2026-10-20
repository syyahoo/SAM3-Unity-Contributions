using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace StrikerPC.Video
{
    public sealed class DroneVisualPoseReceiver : MonoBehaviour
    {
        [SerializeField] private int poseUdpPort = 49004;
        [SerializeField, Range(16, 256)] private int capacity = 120;

        private readonly ConcurrentQueue<string> pendingMessages = new ConcurrentQueue<string>();
        private DronePoseSampleV2[] samples;
        private int count;
        private int writeIndex;
        private long lastSequence;
        private ulong lastSourceTimestampNs;
        private UdpClient receiver;
        private Thread receiveThread;
        private volatile bool running;
        private string pendingSocketWarning;

        [Serializable]
        private sealed class VisualPoseMessage
        {
            public int version;
            public long sequence;
            public ulong sourceTimestampNs;
            public float[] position;
            public float[] rotation;
            public float confidence;
            public int tagCount;
            public float reprojectionError;
            public string source;
        }

        // 시간 정합용 visual pose ring buffer를 준비한다.
        private void Awake()
        {
            EnsureSamples();
        }

        // 플레이 모드에서만 localhost visual pose 수신을 시작한다.
        private void OnEnable()
        {
            if (!Application.isPlaying)
            {
                return;
            }
            StartReceiver();
        }

        // 수신 스레드가 넘긴 JSON을 Unity 메인 스레드에서 검증한다.
        private void Update()
        {
            string warning = Interlocked.Exchange(ref pendingSocketWarning, null);
            if (!string.IsNullOrEmpty(warning))
            {
                Debug.LogWarning($"[Visual Pose] UDP 수신 오류: {warning}", this);
            }
            int processed = 0;
            while (processed < 32 && pendingMessages.TryDequeue(out string json))
            {
                AcceptMessage(json);
                processed++;
            }
        }

        // 지정 시각과 가장 가까운 검증된 visual camera pose만 반환한다.
        public bool TryGetClosestPose(
            ulong sourceTimestampNs,
            ulong maxDeltaNs,
            out DronePoseSampleV2 sample)
        {
            sample = default;
            EnsureSamples();
            if (sourceTimestampNs == 0UL || count == 0)
            {
                return false;
            }

            ulong bestDelta = ulong.MaxValue;
            for (int offset = 0; offset < count; offset++)
            {
                int index = (writeIndex - 1 - offset + samples.Length) % samples.Length;
                DronePoseSampleV2 candidate = samples[index];
                ulong delta = candidate.sourceTimestampNs > sourceTimestampNs
                    ? candidate.sourceTimestampNs - sourceTimestampNs
                    : sourceTimestampNs - candidate.sourceTimestampNs;
                if (delta < bestDelta)
                {
                    bestDelta = delta;
                    sample = candidate;
                }
            }

            return bestDelta <= maxDeltaNs && sample.IsTracked;
        }

        // JSON 계약과 순서를 검증한 뒤 Venue 좌표 pose를 ring buffer에 저장한다.
        private bool AcceptMessage(string json)
        {
            EnsureSamples();
            VisualPoseMessage message;
            try
            {
                message = JsonUtility.FromJson<VisualPoseMessage>(json);
            }
            catch (ArgumentException)
            {
                return false;
            }

            bool newSession = message != null
                && lastSequence > 0
                && message.sequence > 0
                && message.sequence <= 4
                && message.sequence <= lastSequence
                && message.sourceTimestampNs > lastSourceTimestampNs + 250_000_000UL;
            if (message == null
                || message.version != 1
                || (message.sequence <= lastSequence && !newSession)
                || message.sourceTimestampNs == 0UL
                || message.position == null
                || message.position.Length != 3
                || message.rotation == null
                || message.rotation.Length != 4
                || (message.source != "apriltag" && message.source != "anchor")
                || message.tagCount < 0
                || !IsFinite(message.confidence)
                || message.confidence < 0f
                || message.confidence > 1f
                || !IsFinite(message.reprojectionError)
                || message.reprojectionError < 0f)
            {
                return false;
            }

            if (newSession)
            {
                count = 0;
                writeIndex = 0;
            }

            Vector3 position = new Vector3(
                message.position[0],
                message.position[1],
                message.position[2]);
            Quaternion rotation = new Quaternion(
                message.rotation[0],
                message.rotation[1],
                message.rotation[2],
                message.rotation[3]);
            if (!IsFinite(position) || !IsFinite(rotation) || SqrMagnitude(rotation) <= 0.000001f)
            {
                return false;
            }

            lastSequence = message.sequence;
            lastSourceTimestampNs = message.sourceTimestampNs;
            samples[writeIndex] = new DronePoseSampleV2
            {
                frameId = (ulong)message.sequence,
                sourceTimestampNs = message.sourceTimestampNs,
                venuePosition = position,
                venueRotation = Normalize(rotation),
                trackingState = message.source == "apriltag"
                    ? DroneTrackingStateV2.AprilTagLocked
                    : DroneTrackingStateV2.VisualTracked,
                confidence = message.confidence
            };
            writeIndex = (writeIndex + 1) % samples.Length;
            count = Mathf.Min(count + 1, samples.Length);
            return true;
        }

        // Awake 이전 호출에도 ring buffer가 비어 있지 않도록 보장한다.
        private void EnsureSamples()
        {
            if (samples == null || samples.Length == 0)
            {
                samples = new DronePoseSampleV2[Mathf.Max(16, capacity)];
            }
        }

        // localhost UDP 소켓과 전용 수신 스레드를 시작한다.
        private void StartReceiver()
        {
            if (running)
            {
                return;
            }
            receiver = new UdpClient(new IPEndPoint(IPAddress.Loopback, poseUdpPort));
            receiver.Client.ReceiveTimeout = 250;
            running = true;
            receiveThread = new Thread(ReceiveLoop)
            {
                IsBackground = true,
                Name = "StrikerPC Visual Pose Receiver"
            };
            receiveThread.Start();
        }

        // UDP payload를 제한된 큐에 넣고 Unity API 호출은 메인 스레드에 맡긴다.
        private void ReceiveLoop()
        {
            IPEndPoint sender = new IPEndPoint(IPAddress.Loopback, 0);
            while (running)
            {
                try
                {
                    byte[] payload = receiver.Receive(ref sender);
                    if (payload.Length == 0 || payload.Length > 4096)
                    {
                        continue;
                    }
                    while (pendingMessages.Count >= 256)
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

        // quaternion 크기를 1로 정규화한다.
        private static Quaternion Normalize(Quaternion value)
        {
            float inverse = 1f / Mathf.Sqrt(SqrMagnitude(value));
            return new Quaternion(
                value.x * inverse,
                value.y * inverse,
                value.z * inverse,
                value.w * inverse);
        }

        // Vector3 값이 모두 유한한지 확인한다.
        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        // Quaternion 값이 모두 유한한지 확인한다.
        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x)
                && IsFinite(value.y)
                && IsFinite(value.z)
                && IsFinite(value.w);
        }

        // Quaternion 성분의 제곱합을 계산한다.
        private static float SqrMagnitude(Quaternion value)
        {
            return value.x * value.x
                + value.y * value.y
                + value.z * value.z
                + value.w * value.w;
        }

        // float 값이 NaN 또는 Infinity가 아닌지 확인한다.
        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
