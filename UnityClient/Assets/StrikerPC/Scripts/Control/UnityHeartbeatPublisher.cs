using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace StrikerPC.Control
{
    [DisallowMultipleComponent]
    public sealed class UnityHeartbeatPublisher : MonoBehaviour
    {
        [Serializable]
        private sealed class UnityHeartbeatMessage
        {
            public string type = "unity-heartbeat";
            public int schemaVersion = 1;
            public string sessionId;
            public long sequence;
            public long sentAtMs;
            public bool controlConnected;
            public bool unityAirborne;
        }

        [SerializeField] private TelloControlOrchestrator controlOrchestrator;
        [SerializeField, Min(1)] private int heartbeatPort = 49304;
        [SerializeField, Range(1, 30)] private int sendRateHz = 10;

        private UdpClient client;
        private IPEndPoint heartbeatEndpoint;
        private string session;
        private long sequence;
        private float nextSendTime;
        private string pendingError;

        public string Session => session;
        public long Sequence => sequence;

        // Scene 직렬화 변경 없이 현재 조종 상태 공급자를 연결한다.
        public void Configure(TelloControlOrchestrator source)
        {
            controlOrchestrator = source;
        }

        // localhost heartbeat 송신 소켓과 실행 세션을 준비한다.
        private void OnEnable()
        {
            try
            {
                client = new UdpClient(0);
                heartbeatEndpoint = new IPEndPoint(IPAddress.Loopback, heartbeatPort);
                session = Guid.NewGuid().ToString("N");
                sequence = 0;
                nextSendTime = Time.unscaledTime;
            }
            catch (Exception exception)
            {
                pendingError = exception.Message;
                client?.Dispose();
                client = null;
            }
        }

        // 10Hz 주기로 현재 Unity 조종 연결과 비행 상태를 보낸다.
        private void Update()
        {
            if (client != null && Time.unscaledTime >= nextSendTime)
            {
                SendHeartbeat();
                nextSendTime = Time.unscaledTime + 1f / Mathf.Max(1, sendRateHz);
            }

            if (!string.IsNullOrEmpty(pendingError))
            {
                string message = pendingError;
                pendingError = null;
                Debug.LogWarning($"[Unity Heartbeat] {message}", this);
            }
        }

        // heartbeat 송신 소켓을 정리한다.
        private void OnDisable()
        {
            client?.Dispose();
            client = null;
            heartbeatEndpoint = null;
        }

        // 현재 시각과 조종 상태를 직렬화해 Gateway로 보낸다.
        private void SendHeartbeat()
        {
            try
            {
                byte[] payload = Encoding.UTF8.GetBytes(BuildHeartbeatJson(++sequence));
                client.Send(payload, payload.Length, heartbeatEndpoint);
            }
            catch (Exception exception)
            {
                pendingError = exception.Message;
            }
        }

        // heartbeat JSON 계약을 고정 필드로 만든다.
        private string BuildHeartbeatJson(long heartbeatSequence)
        {
            return JsonUtility.ToJson(new UnityHeartbeatMessage
            {
                sessionId = session,
                sequence = heartbeatSequence,
                sentAtMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                controlConnected = controlOrchestrator != null && controlOrchestrator.IsConnected,
                unityAirborne = controlOrchestrator != null && controlOrchestrator.IsAirborne
            });
        }
    }
}
