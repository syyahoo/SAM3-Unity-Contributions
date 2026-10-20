using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace StrikerPC.State
{
    public sealed class TelloStateReceiver : MonoBehaviour
    {
        [SerializeField, Min(1)] private int statePort = 8891;
        [SerializeField, Min(0.1f)] private float disconnectTimeoutSeconds = 1f;

        private readonly object stateLock = new object();
        private UdpClient client;
        private Thread receiverThread;
        private volatile bool receiverRunning;
        private TelloConnectionState latest = new TelloConnectionState();
        private TelloConnectionState pendingNotification;
        private long lastPacketTicks;

        public event Action<TelloConnectionState> StateUpdated;

        // 최신 상태의 복사본을 반환한다.
        public TelloConnectionState Current
        {
            get
            {
                lock (stateLock)
                {
                    TelloConnectionState copy = Clone(latest);
                    copy.stateConnected = IsRecent();
                    return copy;
                }
            }
        }

        // Tello 상태 UDP 수신을 시작한다.
        private void OnEnable()
        {
            if (statePort == 8890)
            {
                statePort = 8891;
                Debug.LogWarning("[Tello State] UDP 8890은 Position Sync 전용이므로 relay 포트 8891을 사용합니다.", this);
            }
            try
            {
                client = new UdpClient(statePort);
                client.Client.ReceiveTimeout = 500;
                receiverRunning = true;
                receiverThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "TelloStateReceiver"
                };
                receiverThread.Start();
            }
            catch (Exception exception)
            {
                Debug.LogError($"[Tello State] UDP {statePort}를 열지 못했습니다: {exception.Message}", this);
            }
        }

        // 상태 UDP 수신을 종료한다.
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
        }

        // 수신 스레드의 최신 상태를 Unity 메인 스레드 이벤트로 전달한다.
        private void Update()
        {
            TelloConnectionState notification = Interlocked.Exchange(ref pendingNotification, null);
            if (notification != null)
            {
                StateUpdated?.Invoke(notification);
            }
        }

        // Tello key:value 상태 패킷을 파싱한다.
        private void ReceiveLoop()
        {
            IPEndPoint sender = null;
            while (receiverRunning)
            {
                try
                {
                    byte[] payload = client.Receive(ref sender);
                    TelloConnectionState parsed = Parse(Encoding.ASCII.GetString(payload));
                    parsed.stateConnected = true;
                    parsed.receivedUtcMilliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                    Interlocked.Exchange(ref lastPacketTicks, System.Diagnostics.Stopwatch.GetTimestamp());
                    lock (stateLock)
                    {
                        latest = parsed;
                    }
                    Interlocked.Exchange(ref pendingNotification, Clone(parsed));
                }
                catch (SocketException exception) when (exception.SocketErrorCode == SocketError.TimedOut)
                {
                }
                catch (Exception)
                {
                    if (!receiverRunning)
                    {
                        return;
                    }
                }
            }
        }

        // SDK 상태 문자열을 필요한 필드만 추출한다.
        private static TelloConnectionState Parse(string message)
        {
            Dictionary<string, string> values = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (string segment in message.Split(';'))
            {
                int separator = segment.IndexOf(':');
                if (separator > 0)
                {
                    values[segment.Substring(0, separator)] = segment.Substring(separator + 1);
                }
            }

            int height = ReadInt(values, "h");
            int tof = ReadInt(values, "tof");
            return new TelloConnectionState
            {
                wifiSignalPercent = ReadOptionalPercent(values, "wifi"),
                batteryPercent = ReadInt(values, "bat"),
                heightCentimeters = height,
                flightTimeSeconds = ReadInt(values, "time"),
                pitch = ReadFloat(values, "pitch"),
                roll = ReadFloat(values, "roll"),
                yaw = ReadFloat(values, "yaw"),
                velocityXCentimetersPerSecond = ReadFloat(values, "vgx"),
                velocityYCentimetersPerSecond = ReadFloat(values, "vgy"),
                velocityZCentimetersPerSecond = ReadFloat(values, "vgz"),
                isFlying = height > 10 || tof > 100
            };
        }

        // 정수 상태값을 안전하게 읽는다.
        private static int ReadInt(IReadOnlyDictionary<string, string> values, string key)
        {
            return values.TryGetValue(key, out string value)
                && int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsed)
                    ? parsed
                    : 0;
        }

        // 실수 상태값을 안전하게 읽는다.
        private static float ReadFloat(IReadOnlyDictionary<string, string> values, string key)
        {
            return values.TryGetValue(key, out string value)
                && float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed)
                    ? parsed
                    : 0f;
        }

        // 선택적인 백분율 필드는 없을 때 미수신 상태로 유지한다.
        private static int ReadOptionalPercent(
            IReadOnlyDictionary<string, string> values,
            string key)
        {
            return values.TryGetValue(key, out string value)
                && int.TryParse(
                    value,
                    NumberStyles.Integer,
                    CultureInfo.InvariantCulture,
                    out int parsed)
                    ? Mathf.Clamp(parsed, 0, 100)
                    : -1;
        }

        // 마지막 상태 패킷이 제한 시간 안인지 확인한다.
        private bool IsRecent()
        {
            long received = Interlocked.Read(ref lastPacketTicks);
            if (received <= 0)
            {
                return false;
            }
            double elapsed = (System.Diagnostics.Stopwatch.GetTimestamp() - received)
                / (double)System.Diagnostics.Stopwatch.Frequency;
            return elapsed <= disconnectTimeoutSeconds;
        }

        // 상태 객체를 외부 변경과 분리해 복사한다.
        private static TelloConnectionState Clone(TelloConnectionState source)
        {
            return new TelloConnectionState
            {
                sdkConnected = source.sdkConnected,
                stateConnected = source.stateConnected,
                videoConnected = source.videoConnected,
                isFlying = source.isFlying,
                wifiSignalPercent = source.wifiSignalPercent,
                batteryPercent = source.batteryPercent,
                heightCentimeters = source.heightCentimeters,
                flightTimeSeconds = source.flightTimeSeconds,
                pitch = source.pitch,
                roll = source.roll,
                yaw = source.yaw,
                velocityXCentimetersPerSecond = source.velocityXCentimetersPerSecond,
                velocityYCentimetersPerSecond = source.velocityYCentimetersPerSecond,
                velocityZCentimetersPerSecond = source.velocityZCentimetersPerSecond,
                receivedUtcMilliseconds = source.receivedUtcMilliseconds
            };
        }
    }
}
