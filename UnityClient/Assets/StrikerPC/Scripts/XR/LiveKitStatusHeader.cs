using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    public sealed class LiveKitStatusHeader : MonoBehaviour
    {
        [SerializeField] private LiveKitAudioBridge communicationBridge;
        [SerializeField] private Text linkStatusText;
        [SerializeField] private Text microphoneStatusText;
        [SerializeField, Min(0.05f)] private float refreshIntervalSeconds = 0.2f;
        [SerializeField] private Color connectedColor = Color.white;
        [SerializeField] private Color waitingColor = Color.white;
        [SerializeField] private Color inactiveColor = Color.white;

        private float nextRefreshAt;

        // Scene의 LiveKit 통신 상태 공급자를 한 번 연결한다.
        private void Awake()
        {
            connectedColor = Color.white;
            waitingColor = Color.white;
            inactiveColor = Color.white;
            communicationBridge ??= FindFirstObjectByType<LiveKitAudioBridge>();
        }

        // 활성화 직후 현재 LINK와 MIC 상태를 표시한다.
        private void OnEnable()
        {
            Refresh();
        }

        // 지정 간격으로 연결과 마이크 상태를 갱신한다.
        private void Update()
        {
            if (Time.unscaledTime < nextRefreshAt)
            {
                return;
            }

            nextRefreshAt = Time.unscaledTime + refreshIntervalSeconds;
            Refresh();
        }

        // Prefab 인스턴스에 Scene 통신 참조를 연결한다.
        public void Configure(LiveKitAudioBridge bridge)
        {
            communicationBridge = bridge;
            Refresh();
        }

        // 현재 LiveKit 상태를 두 개의 상단 표시 Text에 반영한다.
        public void Refresh()
        {
            bool connected = communicationBridge != null && communicationBridge.IsConnected;
            bool microphoneEnabled = connected && communicationBridge.IsMicrophoneEnabled;

            if (linkStatusText != null)
            {
                linkStatusText.text = FormatLinkStatus(connected);
                linkStatusText.color = connected ? connectedColor : waitingColor;
            }

            if (microphoneStatusText != null)
            {
                microphoneStatusText.text = FormatMicrophoneStatus(microphoneEnabled);
                microphoneStatusText.color = microphoneEnabled ? connectedColor : inactiveColor;
            }
        }

        // 연결 여부를 STT 상단 LINK 문구로 변환한다.
        public static string FormatLinkStatus(bool connected)
        {
            return connected ? "LINK  연결됨" : "LINK  연결 대기";
        }

        // 마이크 송출 여부를 STT 상단 MIC 문구로 변환한다.
        public static string FormatMicrophoneStatus(bool enabled)
        {
            return enabled ? "MIC  ON" : "MIC  OFF";
        }
    }
}
