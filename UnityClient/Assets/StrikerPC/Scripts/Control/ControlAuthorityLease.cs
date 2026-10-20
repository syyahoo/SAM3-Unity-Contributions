using System;
using System.Net;
using System.Net.Sockets;

namespace StrikerPC.Control
{
    public sealed class ControlAuthorityLease : IDisposable
    {
        private TcpListener listener;

        public bool IsHeld => listener != null;

        // 로컬 제어 포트를 점유해 중복 송신을 차단한다.
        public bool TryAcquire(int port, out string error)
        {
            if (listener != null)
            {
                error = string.Empty;
                return true;
            }

            try
            {
                listener = new TcpListener(IPAddress.Loopback, port);
                listener.Start(1);
                error = string.Empty;
                return true;
            }
            catch (SocketException exception)
            {
                listener = null;
                error = $"제어 권한 포트 {port}를 점유하지 못했습니다: {exception.Message}";
                return false;
            }
        }

        // 점유한 로컬 제어 포트를 해제한다.
        public void Dispose()
        {
            listener?.Stop();
            listener = null;
        }
    }
}
