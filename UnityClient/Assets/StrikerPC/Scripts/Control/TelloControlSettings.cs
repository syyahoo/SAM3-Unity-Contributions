using UnityEngine;

namespace StrikerPC.Control
{
    [CreateAssetMenu(fileName = "TelloControlSettings", menuName = "Drone Ops/Tello Control Settings")]
    public sealed class TelloControlSettings : ScriptableObject
    {
        [SerializeField] private DroneControlMode controlMode = DroneControlMode.PythonRelay;
        [SerializeField] private string telloIp = "192.168.10.1";
        [SerializeField, Min(1)] private int telloCommandPort = 8889;
        [SerializeField, Min(1)] private int relayPort = 49200;
        [SerializeField, Min(1)] private int authorityPort = 49199;
        [SerializeField] private string sharedControlMemoryName = "TelloPilotControl_v1";
        [SerializeField, Min(0.1f)] private float sharedHandshakeTimeoutSeconds = 0.5f;
        [SerializeField, Min(0.1f)] private float sharedHeartbeatTimeoutSeconds = 0.25f;
        [SerializeField, Min(0.5f)] private float sharedCommandTimeoutSeconds = 8f;
        [Tooltip("Python Relay를 대기 경로로 유지한 채 SharedMemory 제어를 우선 사용합니다.")]
        [SerializeField] private bool preferSharedMemoryWithRelayFallback = true;
        [SerializeField, Range(10, 120)] private int sendRateHz = 60;
        [SerializeField, Range(1, 100)] private int maximumAxis = 100;
        [SerializeField] private bool manageTelloSession;

        public DroneControlMode ControlMode => controlMode;
        public string TelloIp => telloIp;
        public int TelloCommandPort => telloCommandPort;
        public int RelayPort => relayPort;
        public int AuthorityPort => authorityPort;
        public string SharedControlMemoryName => sharedControlMemoryName;
        public float SharedHandshakeTimeoutSeconds => sharedHandshakeTimeoutSeconds;
        public float SharedHeartbeatTimeoutSeconds => sharedHeartbeatTimeoutSeconds;
        public float SharedCommandTimeoutSeconds => sharedCommandTimeoutSeconds;
        public bool PreferSharedMemoryWithRelayFallback => preferSharedMemoryWithRelayFallback;
        public int SendRateHz => sendRateHz;
        public int MaximumAxis => maximumAxis;
        public bool ManageTelloSession => manageTelloSession;
    }
}
