using System.Globalization;
using StrikerPC.State;
using StrikerPC.Video;
using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    [DefaultExecutionOrder(100)]
    public sealed class FpvFlightGuideHud : MonoBehaviour
    {
        [Header("Scene Sources")]
        [SerializeField] private TelloStateReceiver stateReceiver;
        [SerializeField] private PilotSharedFrameTexture frameSource;
        [SerializeField] private DronePoseHistoryProvider poseHistory;
        [SerializeField] private PilotFocusModeController focusModeController;

        [Header("Prefab Visuals")]
        [SerializeField] private RectTransform horizonLine;
        [SerializeField] private Image horizonImage;
        [SerializeField] private Text secondaryStatusText;
        [SerializeField] private Text precisionModeText;
        [SerializeField] private Text speedValueText;
        [SerializeField] private Text altitudeValueText;
        [SerializeField] private FpvAttitudeIndicator attitudeIndicator;

        [Header("Guide Settings")]
        [SerializeField] private float horizonVerticalOffsetPixels = 300f;
        [SerializeField] private Color normalColor = Color.white;
        [SerializeField] private Color warningColor = Color.white;

        // Prefab 인스턴스의 외부 Scene 참조만 연결한다.
        public void ConfigureSceneSources(
            TelloStateReceiver telemetry,
            PilotSharedFrameTexture sharedFrame,
            DronePoseHistoryProvider poses,
            PilotFocusModeController focusMode)
        {
            stateReceiver = telemetry;
            frameSource = sharedFrame;
            poseHistory = poses;
            focusModeController = focusMode;
        }

        // Scene 복제나 누락 참조에서도 기존 컴포넌트만 한 번 찾아 복구한다.
        private void Awake()
        {
            normalColor = Color.white;
            warningColor = Color.white;
            stateReceiver ??= FindFirstObjectByType<TelloStateReceiver>();
            frameSource ??= FindFirstObjectByType<PilotSharedFrameTexture>();
            poseHistory ??= FindFirstObjectByType<DronePoseHistoryProvider>();
            focusModeController ??= FindFirstObjectByType<PilotFocusModeController>();
        }

        // 활성화 직후 직렬화된 표시를 현재 상태로 맞춘다.
        private void OnEnable()
        {
            RefreshGuide();
        }

        // 최신 비행·영상·추적 상태를 Prefab UI에 반영한다.
        private void Update()
        {
            RefreshGuide();
        }

        // 수평선과 상태 텍스트를 한 프레임 단위로 갱신한다.
        private void RefreshGuide()
        {
            TelloConnectionState state = stateReceiver?.Current;
            bool telemetryAvailable = state?.stateConnected == true;
            bool focusActive = focusModeController != null && focusModeController.IsFocused;
            UpdateHorizon(state, telemetryAvailable);
            UpdateFlightTapes(state, telemetryAvailable);
            if (attitudeIndicator != null)
            {
                attitudeIndicator.SetAttitude(
                    state?.pitch ?? 0f,
                    state?.roll ?? 0f,
                    telemetryAvailable);
            }

            if (secondaryStatusText != null)
            {
                secondaryStatusText.text = FormatSecondaryStatus(frameSource);
                secondaryStatusText.color = Color.white;
            }

            if (precisionModeText != null)
            {
                precisionModeText.gameObject.SetActive(focusActive);
            }
        }

        // 고정된 상단 기준점에서 센서 Roll의 반대 방향으로 실제 수평선을 표시한다.
        private void UpdateHorizon(TelloConnectionState state, bool available)
        {
            if (horizonLine == null)
            {
                return;
            }

            horizonLine.gameObject.SetActive(available);
            if (!available)
            {
                return;
            }

            horizonLine.anchoredPosition = new Vector2(0f, horizonVerticalOffsetPixels);
            horizonLine.localRotation = Quaternion.Euler(0f, 0f, -state.roll);
            if (horizonImage != null)
            {
                horizonImage.color = normalColor;
            }
        }

        // 데드맨이 활성화된 이동 입력이 있을 때만 예상 이동경로를 표시한다.
        public static bool ShouldShowPredictedPath(
            bool controlActive,
            Vector4 controlAxes,
            float minimumInput)
        {
            Vector3 translation = new Vector3(
                controlAxes.x,
                controlAxes.z,
                controlAxes.y);
            return controlActive
                && translation.magnitude >= Mathf.Max(0f, minimumInput);
        }

        // 기존 호출부와의 소스 호환성을 유지한다.
        public static bool ShouldShowPredictedPath(
            bool _,
            bool telemetryAvailable,
            float speedMetersPerSecond,
            float minimumSpeedMetersPerSecond)
        {
            return telemetryAvailable
                && speedMetersPerSecond >= Mathf.Max(0f, minimumSpeedMetersPerSecond);
        }

        // 무입력 전방점에 이동·회전 예측을 더해 화면상의 비행경로 마커 위치를 계산한다.
        public static Vector2 CalculateFlightPathMarkerPosition(
            Vector4 controlAxes,
            float predictionSeconds,
            float maximumHorizontalSpeed,
            float maximumVerticalSpeed,
            float maximumYawRateDegrees,
            float cameraDownTilt,
            float verticalFieldOfView,
            float viewportHeight,
            float referenceDistance,
            float maximumPixels)
        {
            Vector3 predictedDisplacement = CalculatePredictedBodyDisplacement(
                controlAxes,
                predictionSeconds,
                maximumHorizontalSpeed,
                maximumVerticalSpeed,
                maximumYawRateDegrees);
            float predictedYawDegrees = controlAxes.w
                * Mathf.Max(0f, maximumYawRateDegrees)
                * Mathf.Max(0f, predictionSeconds);
            Vector3 predictedForwardPoint = Quaternion.Euler(
                    0f,
                    predictedYawDegrees,
                    0f)
                * Vector3.forward
                * Mathf.Max(0.1f, referenceDistance);
            Vector3 aimPoint = predictedForwardPoint + predictedDisplacement;
            if (aimPoint.sqrMagnitude < Mathf.Epsilon)
            {
                aimPoint = Vector3.forward;
            }

            return ProjectBodyPoint(
                aimPoint,
                cameraDownTilt,
                verticalFieldOfView,
                viewportHeight,
                maximumPixels);
        }

        // 기체 기준의 전방점을 카메라 하향각과 FOV에 맞춰 HUD 좌표로 투영한다.
        public static Vector2 ProjectBodyPoint(
            Vector3 bodyPoint,
            float cameraDownTilt,
            float verticalFieldOfView,
            float viewportHeight,
            float maximumPixels)
        {
            float tiltRadians = Mathf.Clamp(cameraDownTilt, 0f, 89f) * Mathf.Deg2Rad;
            float cosine = Mathf.Cos(tiltRadians);
            float sine = Mathf.Sin(tiltRadians);
            float cameraRight = bodyPoint.x;
            float cameraUp = bodyPoint.y * cosine + bodyPoint.z * sine;
            float cameraForward = bodyPoint.z * cosine - bodyPoint.y * sine;
            float safeMaximumPixels = Mathf.Max(0f, maximumPixels);
            Vector2 projected;

            if (cameraForward > 0.01f)
            {
                float safeFov = Mathf.Clamp(verticalFieldOfView, 1f, 179f);
                float focalPixels = Mathf.Max(1f, viewportHeight) * 0.5f
                    / Mathf.Tan(safeFov * 0.5f * Mathf.Deg2Rad);
                projected = new Vector2(cameraRight, cameraUp)
                    * (focalPixels / cameraForward);
            }
            else
            {
                Vector2 screenPlaneDirection = new Vector2(cameraRight, cameraUp);
                projected = screenPlaneDirection.sqrMagnitude > Mathf.Epsilon
                    ? screenPlaneDirection.normalized * safeMaximumPixels
                    : Vector2.zero;
            }

            return Vector2.ClampMagnitude(projected, safeMaximumPixels);
        }

        // 입력 축과 동시 회전을 이용해 예측 시간 뒤의 기체 기준 변위를 계산한다.
        public static Vector3 CalculatePredictedBodyDisplacement(
            Vector4 controlAxes,
            float predictionSeconds,
            float maximumHorizontalSpeed,
            float maximumVerticalSpeed,
            float maximumYawRateDegrees)
        {
            float seconds = Mathf.Max(0f, predictionSeconds);
            float right = controlAxes.x * Mathf.Max(0f, maximumHorizontalSpeed) * seconds;
            float forward = controlAxes.y * Mathf.Max(0f, maximumHorizontalSpeed) * seconds;
            float up = controlAxes.z * Mathf.Max(0f, maximumVerticalSpeed) * seconds;
            float averageYawRadians = controlAxes.w
                * Mathf.Max(0f, maximumYawRateDegrees)
                * seconds
                * 0.5f
                * Mathf.Deg2Rad;
            float cosine = Mathf.Cos(averageYawRadians);
            float sine = Mathf.Sin(averageYawRadians);
            return new Vector3(
                right * cosine + forward * sine,
                up,
                forward * cosine - right * sine);
        }

        // 카메라 하향각과 FOV를 반영해 기체 변위를 HUD 화면 좌표로 투영한다.
        public static Vector2 ProjectPredictedDisplacement(
            Vector3 bodyDisplacement,
            float cameraDownTilt,
            float verticalFieldOfView,
            float viewportHeight,
            float referenceDistance,
            float maximumPixels)
        {
            float tiltRadians = Mathf.Clamp(cameraDownTilt, 0f, 89f) * Mathf.Deg2Rad;
            float cosine = Mathf.Cos(tiltRadians);
            float sine = Mathf.Sin(tiltRadians);
            float cameraRight = bodyDisplacement.x;
            float cameraUp = bodyDisplacement.y * cosine + bodyDisplacement.z * sine;
            float cameraForward = bodyDisplacement.z * cosine - bodyDisplacement.y * sine;
            float safeMaximumPixels = Mathf.Max(0f, maximumPixels);
            Vector2 projectedDirection;

            if (cameraForward > 0.01f)
            {
                float safeFov = Mathf.Clamp(verticalFieldOfView, 1f, 179f);
                float focalPixels = Mathf.Max(1f, viewportHeight) * 0.5f
                    / Mathf.Tan(safeFov * 0.5f * Mathf.Deg2Rad);
                projectedDirection = new Vector2(cameraRight, cameraUp)
                    * (focalPixels / cameraForward);
            }
            else
            {
                Vector2 screenPlaneDirection = new Vector2(cameraRight, cameraUp);
                projectedDirection = screenPlaneDirection.sqrMagnitude > Mathf.Epsilon
                    ? screenPlaneDirection.normalized * safeMaximumPixels
                    : Vector2.zero;
            }

            float distanceScale = Mathf.Clamp01(
                bodyDisplacement.magnitude / Mathf.Max(0.1f, referenceDistance));
            return Vector2.ClampMagnitude(projectedDirection, safeMaximumPixels) * distanceScale;
        }

        // 좌측 상태 영역에 표시할 이동 방향 문자열을 만든다.
        public static string FormatPrimaryStatus(TelloConnectionState state)
        {
            if (state?.stateConnected != true)
            {
                return "MOVE --";
            }

            float horizontalSpeed = CalculateHorizontalSpeed(state);
            string motion = horizontalSpeed < 0.05f
                ? "MOVE HOLD"
                : string.Format(
                    CultureInfo.InvariantCulture,
                    "MOVE {0:000} deg",
                    CalculateMotionDirectionDegrees(
                        state.velocityXCentimetersPerSecond,
                        state.velocityYCentimetersPerSecond));
            return motion;
        }

        // 속도와 고도를 좌우의 현재값 박스에 표시한다.
        private void UpdateFlightTapes(TelloConnectionState state, bool available)
        {
            float speed = available ? state.SpeedMetersPerSecond : 0f;
            float altitude = available ? state.heightCentimeters / 100f : 0f;
            UpdateValue(speedValueText, speed, "F2", available);
            UpdateValue(altitudeValueText, altitude, "F1", available);
        }

        // 텔레메트리 연결 상태에 맞춰 현재값 박스를 갱신한다.
        private void UpdateValue(
            Text valueText,
            float currentValue,
            string format,
            bool available)
        {
            Color color = available ? normalColor : warningColor;
            if (valueText != null)
            {
                valueText.text = available
                    ? currentValue.ToString(format, CultureInfo.InvariantCulture)
                    : "--";
                valueText.color = color;
            }
        }

        // 현재값 근처 눈금이 아래에서 위로 증가하도록 개별 값을 계산한다.
        public static float CalculateTapeTickValue(
            float currentValue,
            float step,
            int index,
            int count)
        {
            int safeCount = Mathf.Max(1, count);
            int safeIndex = Mathf.Clamp(index, 0, safeCount - 1);
            float safeStep = Mathf.Max(0.01f, step);
            float snappedValue = Mathf.Round(Mathf.Max(0f, currentValue) / safeStep) * safeStep;
            float tickValue = snappedValue + (safeIndex - safeCount / 2) * safeStep;
            return Mathf.Max(0f, tickValue);
        }

        // 우측 상태 영역에 영상 지연만 표시한다.
        public static string FormatSecondaryStatus(PilotSharedFrameTexture sharedFrame)
        {
            return sharedFrame != null
                && sharedFrame.LastFrameId > 0
                && sharedFrame.LastSourceTimestampNs > 0UL
                    ? string.Format(
                        CultureInfo.InvariantCulture,
                        "LATENCY {0:F0} ms",
                        sharedFrame.LastFrameAgeMilliseconds)
                    : "LATENCY --";
        }

        // Tello 수평 속도 성분을 초당 미터 단위로 변환한다.
        public static float CalculateHorizontalSpeed(TelloConnectionState state)
        {
            if (state == null)
            {
                return 0f;
            }

            return Mathf.Sqrt(
                state.velocityXCentimetersPerSecond * state.velocityXCentimetersPerSecond
                + state.velocityYCentimetersPerSecond * state.velocityYCentimetersPerSecond)
                / 100f;
        }

        // 전진을 0도로 두고 Tello 기체 기준 수평 이동 방향을 계산한다.
        public static float CalculateMotionDirectionDegrees(float rightSpeed, float forwardSpeed)
        {
            return Mathf.Repeat(Mathf.Atan2(rightSpeed, forwardSpeed) * Mathf.Rad2Deg, 360f);
        }

        // 추적 상태를 좁은 FPV HUD용 고정 문자열로 변환한다.
        public static string GetTrackingLabel(DroneTrackingStateV2 state)
        {
            return state switch
            {
                DroneTrackingStateV2.AprilTagLocked => "APRIL",
                DroneTrackingStateV2.VisualTracked => "VISUAL",
                DroneTrackingStateV2.SensorPredicted => "PREDICT",
                DroneTrackingStateV2.Uncertain => "UNCERTAIN",
                _ => "INIT"
            };
        }
    }
}
