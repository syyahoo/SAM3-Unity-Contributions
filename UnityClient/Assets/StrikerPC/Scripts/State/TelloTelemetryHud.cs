using System.Globalization;
using StrikerPC.Control;
using StrikerPC.Video;
using StrikerPC.XR;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

namespace StrikerPC.State
{
    [DisallowMultipleComponent]
    public sealed class TelloTelemetryHud : MonoBehaviour
    {
        [SerializeField] private TelloStateReceiver stateReceiver;
        [SerializeField] private Text telemetryText;
        [SerializeField, Min(0.05f)] private float refreshIntervalSeconds = 0.2f;
        [SerializeField] private TelloControlOrchestrator controlOrchestrator;
        [SerializeField] private ContinuousDronePoseReceiver poseReceiver;
        [SerializeField] private LiveKitAudioBridge communicationBridge;
        [SerializeField] private Text batteryValueText;
        [FormerlySerializedAs("remainingValueText")]
        [SerializeField] private Text wifiValueText;
        [SerializeField] private Text altitudeValueText;
        [SerializeField] private Text speedValueText;
        [SerializeField] private Text linkValueText;
        [SerializeField] private Text microphoneValueText;
        [SerializeField] private Button microphoneToggleButton;
        [SerializeField] private Image batteryFillImage;

        private DroneAnchorOverlay anchorOverlay;

        private float nextRefreshAt;

        // Scene에 미리 배치된 텔레메트리 HUD를 찾아 수신기를 연결한다.
        public static TelloTelemetryHud EnsureCreated(Transform parent, Transform receiverHost)
        {
            if (parent == null)
            {
                return null;
            }

            TelloTelemetryHud hud = parent.GetComponentInChildren<TelloTelemetryHud>(true);
            if (hud != null)
            {
                TelloStateReceiver receiver = receiverHost != null
                    ? receiverHost.GetComponentInChildren<TelloStateReceiver>(true)
                    : null;
                hud.Configure(receiver, hud.telemetryText);
            }

            return hud;
        }

        // 연결된 수신기와 표시 Text를 교체한다.
        public void Configure(TelloStateReceiver receiver, Text targetText)
        {
            stateReceiver = receiver;
            telemetryText = targetText;
            Refresh();
        }

        // 시안의 6칸 정보 타일과 실제 통신 상태를 연결한다.
        public void ConfigureInfoTiles(
            LiveKitAudioBridge bridge,
            Text battery,
            Text wifi,
            Text altitude,
            Text speed,
            Text link,
            Text microphone,
            Image batteryFill)
        {
            ConfigureInfoTiles(
                bridge,
                battery,
                wifi,
                altitude,
                speed,
                link,
                microphone,
                microphoneToggleButton,
                batteryFill);
        }

        // 시안형 정보 타일과 마이크 클릭 버튼을 실제 통신 상태에 연결한다.
        public void ConfigureInfoTiles(
            LiveKitAudioBridge bridge,
            Text battery,
            Text wifi,
            Text altitude,
            Text speed,
            Text link,
            Text microphone,
            Button microphoneButton,
            Image batteryFill)
        {
            communicationBridge = bridge;
            batteryValueText = battery;
            wifiValueText = wifi;
            altitudeValueText = altitude;
            speedValueText = speed;
            linkValueText = link;
            microphoneValueText = microphone;
            microphoneToggleButton = microphoneButton;
            batteryFillImage = batteryFill;
            Refresh();
        }

        // MIC 타일 클릭을 기존 LiveKit 마이크 토글 경로로 전달한다.
        public void ToggleMicrophone()
        {
            communicationBridge?.ToggleMicrophone();
            Refresh();
        }

        // 실제 활성 조종 경로를 한 줄 드론 정보에 함께 표시한다.
        public void ConfigureControl(TelloControlOrchestrator orchestrator)
        {
            controlOrchestrator = orchestrator;
            Refresh();
        }

        // 시각 지도 상태를 표시할 통합 pose 수신기를 연결한다.
        public void ConfigurePose(ContinuousDronePoseReceiver receiver)
        {
            poseReceiver = receiver;
            Refresh();
        }

        // FPV 추적 손실 폴백 상태를 Scene 참조 변경 없이 HUD에 연결한다.
        public void ConfigureAnchorOverlay(DroneAnchorOverlay overlay)
        {
            anchorOverlay = overlay;
            Refresh();
        }

        // 지정 간격으로 연결 상태와 최신 값을 HUD에 반영한다.
        private void Update()
        {
            if (Time.unscaledTime < nextRefreshAt)
            {
                return;
            }

            nextRefreshAt = Time.unscaledTime + refreshIntervalSeconds;
            Refresh();
        }

        // 현재 텔레메트리 문자열을 갱신한다.
        private void Refresh()
        {
            TelloConnectionState state = stateReceiver != null
                ? stateReceiver.Current
                : new TelloConnectionState();
            if (telemetryText != null)
            {
                telemetryText.text = stateReceiver == null
                    ? "WIFI --"
                    : FormatTelemetry(state);
            }

            SetText(batteryValueText, stateReceiver == null ? "--%" : $"{state.batteryPercent}%");
            SetText(wifiValueText, FormatWifiSignal(stateReceiver != null, state));
            SetText(altitudeValueText, stateReceiver == null ? "-- m" : $"{state.heightCentimeters / 100f:0.0} m");
            SetText(speedValueText, stateReceiver == null ? "-- m/s" : $"{state.SpeedMetersPerSecond:0.0} m/s");

            bool linked = communicationBridge != null
                ? communicationBridge.IsConnected
                : state.stateConnected;
            bool microphoneOn = communicationBridge != null
                && communicationBridge.IsMicrophoneEnabled;
            SetText(linkValueText, linked ? "● 연결됨" : "● 미연결");
            SetText(
                microphoneValueText,
                communicationBridge == null
                    ? "● 사용 불가"
                    : !communicationBridge.IsConnected
                        ? "● 연결 대기"
                        : microphoneOn
                            ? "● 켜짐"
                            : "● 꺼짐");
            if (linkValueText != null)
            {
                linkValueText.color = Color.white;
            }
            if (microphoneValueText != null)
            {
                microphoneValueText.color = Color.white;
            }
            if (microphoneToggleButton != null)
            {
                microphoneToggleButton.interactable = communicationBridge != null
                    && communicationBridge.IsConnected;
            }
            UpdateBatteryFill(stateReceiver == null ? -1 : state.batteryPercent);
        }

        // 최신 상태 패킷의 실제 Wi-Fi 신호 강도를 백분율로 표시한다.
        private static string FormatWifiSignal(bool hasReceiver, TelloConnectionState state)
        {
            if (!hasReceiver
                || !state.stateConnected
                || state.wifiSignalPercent < 0)
            {
                return "--%";
            }

            return $"{Mathf.Clamp(state.wifiSignalPercent, 0, 100)}%";
        }

        // 배터리 바와 경고색을 현재 잔량에 맞춘다.
        private void UpdateBatteryFill(int batteryPercent)
        {
            if (batteryFillImage == null)
            {
                return;
            }

            batteryFillImage.fillAmount = Mathf.Clamp01(batteryPercent / 100f);
            batteryFillImage.color = batteryPercent <= 20
                ? new Color32(217, 72, 63, 255)
                : batteryPercent <= 50
                    ? new Color32(224, 163, 58, 255)
                    : new Color32(79, 169, 124, 255);
        }

        // 선택적인 타일 Text에 문자열을 안전하게 반영한다.
        private static void SetText(Text target, string value)
        {
            if (target != null)
            {
                target.text = value;
            }
        }

        // 현재 위치 소스를 센서 기반 운용 상태 문구로 변환한다.
        private static string FormatMapStatus(
            ContinuousDronePoseData pose,
            bool _)
        {
            return pose.state switch
            {
                ContinuousTrackingState.AprilTagLocked => "APRILTAG ORIGIN",
                ContinuousTrackingState.SensorPredicted => "SENSOR POSITION · APPROX",
                ContinuousTrackingState.VisualTracked => "SENSOR POSITION · APPROX",
                ContinuousTrackingState.Uncertain => "POSITION UNAVAILABLE",
                _ => "WAITING FOR APRILTAG ORIGIN"
            };
        }

        // 배터리·높이·자세 값을 한 줄 HUD 문자열로 만든다.
        private static string FormatTelemetry(TelloConnectionState state)
        {
            string wifi = state.stateConnected && state.wifiSignalPercent >= 0
                ? $"WIFI {Mathf.Clamp(state.wifiSignalPercent, 0, 100)}%"
                : "WIFI --";
            return string.Format(
                CultureInfo.InvariantCulture,
                "{0}  |  BAT {1}%  |  ALT {2:0.00} m  |  P {3:0.#}°  R {4:0.#}°",
                wifi,
                state.batteryPercent,
                state.heightCentimeters / 100f,
                state.pitch,
                state.roll);
        }
    }
}
