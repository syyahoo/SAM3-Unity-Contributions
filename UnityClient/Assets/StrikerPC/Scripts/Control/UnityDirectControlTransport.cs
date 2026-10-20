using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace StrikerPC.Control
{
    public sealed class UnityDirectControlTransport : IDroneControlTransport
    {
        private readonly TelloControlSettings settings;
        private readonly ControlAuthorityLease authorityLease = new ControlAuthorityLease();
        private UdpClient client;
        private IPEndPoint telloEndpoint;

        public bool IsConnected { get; private set; }
        public string LastError { get; private set; } = string.Empty;

        // 직접 전송 설정을 보관한다.
        public UnityDirectControlTransport(TelloControlSettings settings)
        {
            this.settings = settings;
        }

        // Tello 명령 소켓과 단독 제어 권한을 준비한다.
        public bool Connect()
        {
            if (!authorityLease.TryAcquire(settings.AuthorityPort, out string error))
            {
                LastError = error;
                return false;
            }

            try
            {
                telloEndpoint = new IPEndPoint(IPAddress.Parse(settings.TelloIp), settings.TelloCommandPort);
                client = new UdpClient(0);
                client.Connect(telloEndpoint);
                client.Client.ReceiveTimeout = 3000;
                IsConnected = true;

                if (settings.ManageTelloSession)
                {
                    if (!SendAsciiWithAck("command"))
                    {
                        throw new SocketException((int)SocketError.TimedOut);
                    }

                    Thread.Sleep(100);
                    if (!SendAsciiWithAck("streamon"))
                    {
                        throw new SocketException((int)SocketError.TimedOut);
                    }
                }

                return true;
            }
            catch (Exception exception)
            {
                LastError = exception.Message;
                Dispose();
                return false;
            }
        }

        // 최신 조종 프레임을 Tello SDK 명령으로 보낸다.
        public bool Send(in DroneControlFrame frame, out long transportTicks)
        {
            transportTicks = Stopwatch.GetTimestamp();
            if (!IsConnected)
            {
                return false;
            }

            string command = frame.Command switch
            {
                DroneControlCommand.Takeoff => "takeoff",
                DroneControlCommand.Land => "land",
                DroneControlCommand.Emergency => "emergency",
                _ => $"rc {frame.LeftRight} {frame.ForwardBack} {frame.UpDown} {frame.Yaw}"
            };

            try
            {
                if (frame.Command is DroneControlCommand.Takeoff or DroneControlCommand.Land or DroneControlCommand.Emergency)
                {
                    if (!SendAsciiWithAck(command))
                    {
                        LastError = $"Tello가 {command} 명령을 확인하지 않았습니다.";
                        return false;
                    }
                }
                else
                {
                    SendAscii(command);
                }
                transportTicks = Stopwatch.GetTimestamp();
                return true;
            }
            catch (SocketException exception)
            {
                LastError = exception.Message;
                return false;
            }
        }

        // ASCII SDK 명령을 UDP로 보낸다.
        private void SendAscii(string command)
        {
            byte[] payload = Encoding.ASCII.GetBytes(command);
            client.Send(payload, payload.Length);
        }

        // 확정 명령의 Tello 응답을 기다린다.
        private bool SendAsciiWithAck(string command)
        {
            SendAscii(command);
            IPEndPoint sender = null;
            byte[] response = client.Receive(ref sender);
            string message = Encoding.ASCII.GetString(response).Trim();
            return message.Equals("ok", StringComparison.OrdinalIgnoreCase);
        }

        // 중립 입력 후 네트워크 자원을 해제한다.
        public void Dispose()
        {
            if (IsConnected)
            {
                try
                {
                    SendAscii("rc 0 0 0 0");
                }
                catch (Exception)
                {
                }
            }

            IsConnected = false;
            client?.Dispose();
            client = null;
            authorityLease.Dispose();
        }
    }
}
