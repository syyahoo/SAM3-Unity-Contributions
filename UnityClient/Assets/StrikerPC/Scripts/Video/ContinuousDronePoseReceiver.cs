using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

namespace StrikerPC.Video
{
    public enum ContinuousTrackingState : byte
    {
        Initializing = 0,
        AprilTagLocked = 1,
        VisualTracked = 2,
        SensorPredicted = 3,
        Uncertain = 4
    }

    [Serializable]
    public struct ContinuousDronePoseData
    {
        public ulong frameId;
        public double timestamp;
        public ContinuousTrackingState state;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 velocity;
        public Vector3 positionStdDev;
        public float yawStdDev;
        public int visualAnchorCount;
        public int detectedTagCount;
        public float reprojectionError;
        public float confidence;
        public bool localMapReady;
        public bool localMapDegraded;
        public bool relocalizing;
        public bool sessionMap;

        public bool IsTracked => state is ContinuousTrackingState.AprilTagLocked
            or ContinuousTrackingState.VisualTracked
            or ContinuousTrackingState.SensorPredicted;
    }

    public sealed class ContinuousDronePoseReceiver : MonoBehaviour
    {
        private const uint Tvp1Magic = 0x31505654;
        private const uint Tvp2Magic = 0x32505654;
        private const int Tvp1Size = 64;
        private const int Tvp2Size = 88;
        private const byte LocalMapReadyFlag = 1 << 0;
        private const byte LocalMapDegradedFlag = 1 << 1;
        private const byte RelocalizingFlag = 1 << 2;
        private const byte SessionMapFlag = 1 << 3;

        [SerializeField, Min(1)] private int poseUdpPort = 49001;
        [SerializeField] private Transform poseTarget;
        [SerializeField, Min(0.001f)] private float smoothingSeconds = 0.08f;
        [SerializeField, Min(0.05f)] private float staleTimeoutSeconds = 0.5f;

        private readonly object poseLock = new object();
        private UdpClient udpClient;
        private Thread receiverThread;
        private volatile bool receiverRunning;
        private ContinuousDronePoseData pendingPose;
        private bool hasPendingPose;
        private Vector3 targetPosition;
        private Quaternion targetRotation = Quaternion.identity;
        private float lastPoseReceivedAt = float.NegativeInfinity;

        public ContinuousDronePoseData LatestPose { get; private set; }
        public bool HasReceivedPose { get; private set; }

        // 기존 Vision Prefab의 pose target을 유지해 연결한다.
        public void Configure(Transform target)
        {
            poseTarget = target;
        }

        // TVP2 pose UDP 수신 thread를 시작한다.
        private void OnEnable()
        {
            receiverRunning = true;
            receiverThread = new Thread(ReceiveLoop)
            {
                IsBackground = true,
                Name = "StrikerPCContinuousPoseReceiver"
            };
            receiverThread.Start();
        }

        // 최신 pose를 대상 Transform에 부드럽게 적용한다.
        private void Update()
        {
            lock (poseLock)
            {
                if (hasPendingPose)
                {
                    LatestPose = pendingPose;
                    hasPendingPose = false;
                    HasReceivedPose = true;
                    lastPoseReceivedAt = Time.unscaledTime;
                    if (LatestPose.IsTracked)
                    {
                        targetPosition = LatestPose.position;
                        targetRotation = LatestPose.rotation;
                    }
                }
            }

            if (HasReceivedPose
                && Time.unscaledTime - lastPoseReceivedAt >= staleTimeoutSeconds)
            {
                ContinuousDronePoseData stalePose = LatestPose;
                stalePose.state = ContinuousTrackingState.Uncertain;
                LatestPose = stalePose;
                HasReceivedPose = false;
            }

            if (poseTarget == null || !HasReceivedPose || !LatestPose.IsTracked)
            {
                return;
            }

            float blend = 1f - Mathf.Exp(
                -Time.unscaledDeltaTime / Mathf.Max(0.001f, smoothingSeconds));
            poseTarget.localPosition = Vector3.Lerp(
                poseTarget.localPosition,
                targetPosition,
                blend);
            poseTarget.localRotation = Quaternion.Slerp(
                poseTarget.localRotation,
                targetRotation,
                blend);
        }

        // pose UDP socket과 thread를 종료한다.
        private void OnDisable()
        {
            receiverRunning = false;
            udpClient?.Close();
            if (receiverThread != null && receiverThread.IsAlive)
            {
                receiverThread.Join(1000);
            }
            receiverThread = null;
            udpClient = null;
        }

        // localhost pose packet을 수신해 최신 sequence만 보관한다.
        private void ReceiveLoop()
        {
            try
            {
                udpClient = new UdpClient(
                    new IPEndPoint(IPAddress.Loopback, poseUdpPort));
                udpClient.Client.ReceiveTimeout = 500;
                IPEndPoint sender = new IPEndPoint(IPAddress.Loopback, 0);
                while (receiverRunning)
                {
                    try
                    {
                        byte[] packet = udpClient.Receive(ref sender);
                        if (TryParsePacket(packet, out ContinuousDronePoseData pose))
                        {
                            lock (poseLock)
                            {
                                if (!hasPendingPose || pose.frameId >= pendingPose.frameId)
                                {
                                    pendingPose = pose;
                                    hasPendingPose = true;
                                }
                            }
                        }
                    }
                    catch (SocketException exception)
                        when (exception.SocketErrorCode == SocketError.TimedOut)
                    {
                    }
                    catch (ObjectDisposedException)
                    {
                        return;
                    }
                }
            }
            catch (Exception exception)
            {
                if (receiverRunning)
                {
                    Debug.LogError(
                        $"[Position Sync] pose UDP 수신 실패: {exception.Message}",
                        this);
                }
            }
        }

        // TVP2와 진단용 TVP1 packet을 검증해 읽는다.
        private static bool TryParsePacket(
            byte[] packet,
            out ContinuousDronePoseData pose)
        {
            pose = default;
            if (packet == null || (packet.Length != Tvp1Size && packet.Length != Tvp2Size))
            {
                return false;
            }

            using MemoryStream stream = new MemoryStream(packet, false);
            using BinaryReader reader = new BinaryReader(stream);
            uint magic = reader.ReadUInt32();
            ushort version = reader.ReadUInt16();
            if (magic == Tvp2Magic && version == 2 && packet.Length == Tvp2Size)
            {
                pose.state = (ContinuousTrackingState)reader.ReadByte();
                byte flags = reader.ReadByte();
                pose.localMapReady = (flags & LocalMapReadyFlag) != 0;
                pose.localMapDegraded = (flags & LocalMapDegradedFlag) != 0;
                pose.relocalizing = (flags & RelocalizingFlag) != 0;
                pose.sessionMap = (flags & SessionMapFlag) != 0;
                pose.frameId = reader.ReadUInt64();
                pose.timestamp = reader.ReadUInt64() / 1_000_000_000d;
                pose.position = ReadVector3(reader);
                pose.rotation = ReadQuaternion(reader);
                pose.velocity = ReadVector3(reader);
                pose.positionStdDev = ReadVector3(reader);
                pose.yawStdDev = reader.ReadSingle();
                pose.confidence = reader.ReadSingle();
                pose.visualAnchorCount = reader.ReadUInt16();
                pose.detectedTagCount = reader.ReadUInt16();
            }
            else if (magic == Tvp1Magic && version == 1 && packet.Length == Tvp1Size)
            {
                byte legacyState = reader.ReadByte();
                pose.state = legacyState == 0
                    ? ContinuousTrackingState.Initializing
                    : legacyState == 1
                        ? ContinuousTrackingState.AprilTagLocked
                        : legacyState == 2
                            ? ContinuousTrackingState.VisualTracked
                            : ContinuousTrackingState.Uncertain;
                reader.ReadByte();
                pose.frameId = reader.ReadUInt64();
                pose.timestamp = reader.ReadDouble();
                pose.position = ReadVector3(reader);
                pose.rotation = ReadQuaternion(reader);
                pose.detectedTagCount = reader.ReadInt32();
                pose.reprojectionError = reader.ReadSingle();
                pose.confidence = reader.ReadSingle();
            }
            else
            {
                return false;
            }

            return pose.state <= ContinuousTrackingState.Uncertain
                && IsFinite(pose.position)
                && IsFinite(pose.rotation)
                && IsFinite(pose.positionStdDev);
        }

        // BinaryReader에서 Vector3를 읽는다.
        private static Vector3 ReadVector3(BinaryReader reader)
        {
            return new Vector3(
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle());
        }

        // BinaryReader에서 Quaternion을 읽는다.
        private static Quaternion ReadQuaternion(BinaryReader reader)
        {
            return new Quaternion(
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle());
        }

        // Vector3 값이 유한한지 확인한다.
        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        // Quaternion 값이 유한한지 확인한다.
        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x)
                && IsFinite(value.y)
                && IsFinite(value.z)
                && IsFinite(value.w);
        }

        // float 값이 유한한지 확인한다.
        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
