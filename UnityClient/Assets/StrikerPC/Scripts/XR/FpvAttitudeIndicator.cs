using UnityEngine;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    public sealed class FpvAttitudeIndicator : MonoBehaviour
    {
        [Header("Prefab Visuals")]
        [SerializeField] private RectTransform rollGroup;
        [SerializeField] private RectTransform pitchGroup;
        [SerializeField] private CanvasGroup canvasGroup;

        [Header("Attitude Settings")]
        [SerializeField, Min(0.1f)] private float pixelsPerDegree = 2f;
        [SerializeField, Range(10f, 45f)] private float maximumDisplayedPitchDegrees = 30f;
        [SerializeField, Min(0f)] private float smoothingSeconds = 0.12f;
        [SerializeField, Range(0f, 1f)] private float unavailableAlpha = 0.4f;

        private float targetPitchDegrees;
        private float targetRollDegrees;
        private float displayedPitchDegrees;
        private float displayedRollDegrees;
        private float pitchVelocity;
        private float rollVelocity;
        private bool initialized;

        // 부모 HUD가 수신한 센서 자세를 목표값으로 전달한다.
        public void SetAttitude(float pitchDegrees, float rollDegrees, bool available)
        {
            targetPitchDegrees = Mathf.Clamp(
                pitchDegrees,
                -maximumDisplayedPitchDegrees,
                maximumDisplayedPitchDegrees);
            targetRollDegrees = Mathf.DeltaAngle(0f, rollDegrees);

            if (canvasGroup != null)
            {
                canvasGroup.alpha = available ? 1f : unavailableAlpha;
            }

            if (initialized)
            {
                return;
            }

            displayedPitchDegrees = targetPitchDegrees;
            displayedRollDegrees = targetRollDegrees;
            initialized = true;
            ApplyVisuals();
        }

        // 활성화 시 직렬화된 초기 자세를 즉시 반영한다.
        private void OnEnable()
        {
            ApplyVisuals();
        }

        // 센서 갱신 사이의 자세 변화를 부드럽게 보간한다.
        private void Update()
        {
            if (!initialized)
            {
                return;
            }

            if (smoothingSeconds <= 0f)
            {
                displayedPitchDegrees = targetPitchDegrees;
                displayedRollDegrees = targetRollDegrees;
            }
            else
            {
                displayedPitchDegrees = Mathf.SmoothDamp(
                    displayedPitchDegrees,
                    targetPitchDegrees,
                    ref pitchVelocity,
                    smoothingSeconds);
                displayedRollDegrees = Mathf.SmoothDampAngle(
                    displayedRollDegrees,
                    targetRollDegrees,
                    ref rollVelocity,
                    smoothingSeconds);
            }

            ApplyVisuals();
        }

        // Roll 부모와 Pitch 자식을 분리해 상하 이동도 기울어진 축을 따르게 한다.
        private void ApplyVisuals()
        {
            if (rollGroup != null)
            {
                rollGroup.localRotation = Quaternion.Euler(
                    0f,
                    0f,
                    CalculateRollAngle(displayedRollDegrees));
            }

            if (pitchGroup != null)
            {
                pitchGroup.anchoredPosition = new Vector2(
                    0f,
                    CalculatePitchOffset(
                        displayedPitchDegrees,
                        pixelsPerDegree,
                        maximumDisplayedPitchDegrees));
            }
        }

        // 기수 상승 시 실제 지평선이 고정 기체 마커 아래로 이동하도록 계산한다.
        public static float CalculatePitchOffset(
            float pitchDegrees,
            float pixelsPerDegree,
            float maximumPitchDegrees)
        {
            float clampedPitch = Mathf.Clamp(
                pitchDegrees,
                -Mathf.Abs(maximumPitchDegrees),
                Mathf.Abs(maximumPitchDegrees));
            return -clampedPitch * Mathf.Max(0f, pixelsPerDegree);
        }

        // 세계 수평선이 기체 Roll의 반대 방향으로 회전하도록 계산한다.
        public static float CalculateRollAngle(float rollDegrees)
        {
            return -Mathf.DeltaAngle(0f, rollDegrees);
        }
    }
}
