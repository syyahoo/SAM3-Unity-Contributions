using UnityEngine;

namespace StrikerPC.Map
{
    public static class AprilTagBoardBearing
    {
        public const string Reference = "apriltag-board-forward";

        private static readonly string[] CardinalDirections =
        {
            "N",
            "NE",
            "E",
            "SE",
            "S",
            "SW",
            "W",
            "NW"
        };

        // AprilTag 보드의 +Z 방향을 북쪽으로 삼아 드론 전방의 시계방향 각도를 구한다.
        public static float FromVenueRotation(
            Quaternion venueRotation,
            float[] boardToVenue)
        {
            Vector3 droneForward = venueRotation * Vector3.forward;
            droneForward.y = 0f;
            if (!IsFinite(droneForward) || droneForward.sqrMagnitude <= 0.000001f)
            {
                return Normalize(venueRotation.eulerAngles.y);
            }
            return FromVenueDirection(droneForward, boardToVenue);
        }

        // Venue 바닥 방향 벡터를 AprilTag 보드 +Z 기준 방위각으로 변환한다.
        public static float FromVenueDirection(
            Vector3 venueDirection,
            float[] boardToVenue)
        {
            Vector3 north = GetHorizontalBoardForward(boardToVenue);
            Vector3 east = Vector3.Cross(Vector3.up, north).normalized;
            venueDirection.y = 0f;
            if (!IsFinite(venueDirection) || venueDirection.sqrMagnitude <= 0.000001f)
            {
                return 0f;
            }

            venueDirection.Normalize();
            float degrees = Mathf.Atan2(
                Vector3.Dot(venueDirection, east),
                Vector3.Dot(venueDirection, north)) * Mathf.Rad2Deg;
            return Normalize(degrees);
        }

        // 임의 각도를 0도 이상 360도 미만으로 정규화한다.
        public static float Normalize(float degrees)
        {
            if (float.IsNaN(degrees) || float.IsInfinity(degrees))
            {
                return 0f;
            }

            float normalized = Mathf.Repeat(degrees, 360f);
            return normalized >= 359.9999f ? 0f : normalized;
        }

        // 방위각을 8방위 문자열로 변환한다.
        public static string GetCardinal(float degrees)
        {
            int index = Mathf.FloorToInt((Normalize(degrees) + 22.5f) / 45f) % 8;
            return CardinalDirections[index];
        }

        // Quest 지도에 표시할 방위 문자열을 만든다.
        public static string Format(float degrees)
        {
            float normalized = Normalize(degrees);
            int roundedDegrees = Mathf.RoundToInt(normalized) % 360;
            return $"{GetCardinal(normalized)} {roundedDegrees:000}°";
        }

        // boardToVenue의 보드 +Z를 Venue 바닥면에 투영한다.
        private static Vector3 GetHorizontalBoardForward(float[] boardToVenue)
        {
            if (boardToVenue == null || boardToVenue.Length != 16)
            {
                return Vector3.forward;
            }

            Vector3 forward = new Vector3(boardToVenue[2], 0f, boardToVenue[10]);
            if (!IsFinite(forward) || forward.sqrMagnitude <= 0.000001f)
            {
                return Vector3.forward;
            }

            return forward.normalized;
        }

        // 좌표 계약에 사용할 벡터가 유한한지 확인한다.
        private static bool IsFinite(Vector3 value)
        {
            return float.IsFinite(value.x)
                && float.IsFinite(value.y)
                && float.IsFinite(value.z);
        }
    }
}
