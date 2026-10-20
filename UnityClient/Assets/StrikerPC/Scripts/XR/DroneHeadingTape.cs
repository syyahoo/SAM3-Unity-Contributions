using StrikerPC.Map;
using StrikerPC.State;
using StrikerPC.Video;
using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.XR
{
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public sealed class DroneHeadingTape : MonoBehaviour
    {
        private const int TickStepDegrees = 10;
        private const float PixelsPerDegree = 5.5f;

        [SerializeField] private TelloStateReceiver stateReceiver;
        [SerializeField] private DronePoseHistoryProvider poseHistory;
        [SerializeField] private VenueMapLoader mapLoader;
        [SerializeField] private Text bearingText;
        [SerializeField] private Text[] tickLabels;

        public float CurrentBearingDegrees { get; private set; }

        // Edit Mode와 런타임 모두에서 직렬화된 눈금 상태를 즉시 맞춘다.
        private void OnEnable()
        {
            Refresh();
        }

        // Scene에 미리 만든 방위 테이프와 회전 데이터 공급자를 연결한다.
        public void Configure(
            TelloStateReceiver receiver,
            DronePoseHistoryProvider history,
            VenueMapLoader loader,
            Text readout,
            Text[] labels)
        {
            stateReceiver = receiver;
            poseHistory = history;
            mapLoader = loader;
            bearingText = readout;
            tickLabels = labels;
            Refresh();
        }

        // 드론 회전과 동일 프레임으로 상단 방위 테이프를 갱신한다.
        private void LateUpdate()
        {
            Refresh();
        }

        // 위치 추적 방위를 우선하고 미수신 구간에는 Tello yaw를 사용한다.
        private float ResolveBearing()
        {
            if (poseHistory != null
                && mapLoader?.Current != null
                && poseHistory.TryGetLatestPose(out DronePoseSampleV2 pose)
                && pose.IsTracked)
            {
                return AprilTagBoardBearing.FromVenueRotation(
                    pose.venueRotation,
                    mapLoader.Current.boardToVenue);
            }

            return stateReceiver != null
                ? AprilTagBoardBearing.Normalize(stateReceiver.Current.yaw)
                : 0f;
        }

        // 현재 방위값과 순환하는 10도 눈금을 화면에 배치한다.
        private void Refresh()
        {
            CurrentBearingDegrees = ResolveBearing();
            if (bearingText != null)
            {
                bearingText.text = AprilTagBoardBearing.Format(CurrentBearingDegrees);
            }
            if (tickLabels == null || tickLabels.Length == 0)
            {
                return;
            }

            int centerIndex = tickLabels.Length / 2;
            int baseTick = Mathf.FloorToInt(CurrentBearingDegrees / TickStepDegrees);
            for (int index = 0; index < tickLabels.Length; index++)
            {
                Text tick = tickLabels[index];
                if (tick == null)
                {
                    continue;
                }

                int degrees = PositiveModulo(
                    (baseTick + index - centerIndex) * TickStepDegrees,
                    360);
                float delta = Mathf.DeltaAngle(CurrentBearingDegrees, degrees);
                tick.enabled = Mathf.Abs(delta) >= 18f;
                tick.rectTransform.anchoredPosition = new Vector2(
                    delta * PixelsPerDegree,
                    0f);
                tick.text = FormatTick(degrees);
            }
        }

        // 4대 방위에는 영문 방위와 정확한 각도를 함께 표시한다.
        public static string FormatTick(int degrees)
        {
            int normalized = PositiveModulo(degrees, 360);
            return normalized switch
            {
                0 => "N\n000",
                90 => "E\n090",
                180 => "S\n180",
                270 => "W\n270",
                _ => $"{normalized:000}"
            };
        }

        // 음수 각도도 0 이상 divisor 미만으로 변환한다.
        private static int PositiveModulo(int value, int divisor)
        {
            return (value % divisor + divisor) % divisor;
        }
    }
}
