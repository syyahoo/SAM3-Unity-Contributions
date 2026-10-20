using System;
using System.Buffers.Binary;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;

namespace StrikerPC.Control
{
    public sealed class PythonRelayControlTransport : IDroneControlTransport
    {
        private const int AckSize = 32;
        private const int HealthProbeIntervalMilliseconds = 500;

        private readonly TelloControlSettings settings;
        private readonly byte[] packet = new byte[ControlPacketCodec.PacketSize];
        private UdpClient client;
        private IPEndPoint relayEndpoint;
        private long nextHealthProbeTicks;

        public bool IsConnected { get; private set; }
        public string LastError { get; private set; } = string.Empty;

        // Python 릴레이 설정을 보관한다.
        public PythonRelayControlTransport(TelloControlSettings settings)
        {
            this.settings = settings;
        }

        // localhost 릴레이 소켓을 준비한다.
        public bool Connect()
        {
            try
            {
                relayEndpoint = new IPEndPoint(IPAddress.Loopback, settings.RelayPort);
                client = new UdpClient(0);
                IsConnected = true;
                DroneControlFrame hello = new DroneControlFrame(0, Stopwatch.GetTimestamp(), 0, 0, 0, 0, DroneControlCommand.Hello);
                if (!SendWithAck(hello, settings.SharedHandshakeTimeoutSeconds))
                {
                    LastError = "Python 제어 릴레이가 응답하지 않습니다.";
                    Dispose();
                    return false;
                }

                nextHealthProbeTicks = Stopwatch.GetTimestamp()
                    + Stopwatch.Frequency * HealthProbeIntervalMilliseconds / 1000;

                return true;
            }
            catch (Exception exception)
            {
                LastError = exception.Message;
                Dispose();
                return false;
            }
        }

        // 고정 크기 조종 패킷을 Python 릴레이로 보낸다.
        public bool Send(in DroneControlFrame frame, out long transportTicks)
        {
            transportTicks = Stopwatch.GetTimestamp();
            if (!IsConnected)
            {
                return false;
            }

            try
            {
                long now = Stopwatch.GetTimestamp();
                if (frame.Command is DroneControlCommand.Rc or DroneControlCommand.Neutral
                    && now >= nextHealthProbeTicks)
                {
                    DroneControlFrame probe = new DroneControlFrame(
                        frame.Sequence,
                        frame.InputTicks,
                        0,
                        0,
                        0,
                        0,
                        DroneControlCommand.Hello);
                    if (!SendWithAck(probe, settings.SharedHandshakeTimeoutSeconds))
                    {
                        LastError = "Python 제어 Relay 상태 확인 ACK가 없습니다.";
                        return false;
                    }
                    nextHealthProbeTicks = Stopwatch.GetTimestamp()
                        + Stopwatch.Frequency * HealthProbeIntervalMilliseconds / 1000;
                }

                Encode(frame);
                client.Send(packet, packet.Length, relayEndpoint);
                transportTicks = Stopwatch.GetTimestamp();
                if (frame.Command is DroneControlCommand.Takeoff or DroneControlCommand.Land or DroneControlCommand.Emergency)
                {
                    return ReceiveAck(frame.Sequence, settings.SharedCommandTimeoutSeconds);
                }

                return true;
            }
            catch (SocketException exception)
            {
                LastError = exception.Message;
                return false;
            }
        }

        // 확정 명령을 보내고 같은 sequence의 ACK를 확인한다.
        private bool SendWithAck(in DroneControlFrame frame, float timeoutSeconds)
        {
            Encode(frame);
            client.Send(packet, packet.Length, relayEndpoint);
            return ReceiveAck(frame.Sequence, timeoutSeconds);
        }

        // Python 릴레이 ACK의 규격과 sequence를 확인한다.
        private bool ReceiveAck(ulong sequence, float timeoutSeconds)
        {
            int previousTimeout = client.Client.ReceiveTimeout;
            client.Client.ReceiveTimeout = TimeoutMilliseconds(timeoutSeconds);
            try
            {
                IPEndPoint sender = null;
                byte[] response = client.Receive(ref sender);
                if (response.Length != AckSize)
                {
                    return false;
                }

                ReadOnlySpan<byte> span = response;
                return BinaryPrimitives.ReadUInt32LittleEndian(span.Slice(0, 4)) == ControlPacketCodec.Magic
                    && BinaryPrimitives.ReadUInt16LittleEndian(span.Slice(4, 2)) == ControlPacketCodec.Version
                    && BinaryPrimitives.ReadUInt16LittleEndian(span.Slice(6, 2)) == 0
                    && BinaryPrimitives.ReadUInt64LittleEndian(span.Slice(8, 8)) == sequence
                    && span[24] == 1;
            }
            finally
            {
                client.Client.ReceiveTimeout = previousTimeout;
            }
        }

        // 초 단위 ACK 제한 시간을 Socket millisecond 값으로 변환한다.
        private static int TimeoutMilliseconds(float timeoutSeconds)
        {
            return Math.Max(1, (int)Math.Ceiling(Math.Max(0.001f, timeoutSeconds) * 1000f));
        }

        // Python struct 규격과 동일한 little-endian 패킷을 만든다.
        private void Encode(in DroneControlFrame frame)
        {
            ControlPacketCodec.Encode(packet, frame);
        }

        // IPC 소켓을 해제한다.
        public void Dispose()
        {
            IsConnected = false;
            client?.Dispose();
            client = null;
        }
    }
}
