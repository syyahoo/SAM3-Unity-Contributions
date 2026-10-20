using System.Collections.Generic;
using UnityEngine;

namespace StrikerPC.Video
{
    public static class DroneCameraProjector
    {
        // 전시장 앵커를 보정된 Tello 영상 픽셀로 투영한다.
        public static bool TryProject(
            Vector3 venuePoint,
            DronePoseSampleV2 cameraPose,
            DroneCameraCalibration calibration,
            out Vector2 pixel,
            out float distance)
        {
            pixel = default;
            distance = 0f;
            if (calibration == null || !calibration.IsValid || !cameraPose.IsTracked)
            {
                return false;
            }

            return TryProject(
                venuePoint,
                cameraPose,
                calibration.Fx,
                calibration.Fy,
                calibration.Cx,
                calibration.Cy,
                calibration.Distortion,
                out pixel,
                out distance);
        }

        // 테스트와 Python parity 검증을 위해 명시적 보정값으로 투영한다.
        public static bool TryProject(
            Vector3 venuePoint,
            DronePoseSampleV2 cameraPose,
            float fx,
            float fy,
            float cx,
            float cy,
            IReadOnlyList<float> distortion,
            out Vector2 pixel,
            out float distance)
        {
            pixel = default;
            distance = 0f;
            if (!cameraPose.IsTracked || fx <= 0f || fy <= 0f)
            {
                return false;
            }

            Vector3 cameraPoint = Quaternion.Inverse(cameraPose.venueRotation)
                * (venuePoint - cameraPose.venuePosition);
            distance = cameraPoint.magnitude;
            if (cameraPoint.z <= 0.001f)
            {
                return false;
            }

            // Vision pose는 OpenCV 영상을 proper quaternion으로 보존하기 위해 영상 오른쪽을 local -X로 둔다.
            float x = -cameraPoint.x / cameraPoint.z;
            float y = -cameraPoint.y / cameraPoint.z;
            ApplyDistortion(ref x, ref y, distortion);
            pixel = new Vector2(
                fx * x + cx,
                fy * y + cy);
            return true;
        }

        // OpenCV radtan 계수를 정규화 영상 좌표에 적용한다.
        private static void ApplyDistortion(ref float x, ref float y, IReadOnlyList<float> distortion)
        {
            if (distortion == null || distortion.Count < 4)
            {
                return;
            }

            float k1 = distortion[0];
            float k2 = distortion[1];
            float p1 = distortion[2];
            float p2 = distortion[3];
            float k3 = distortion.Count > 4 ? distortion[4] : 0f;
            float r2 = x * x + y * y;
            float radial = 1f + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            float distortedX = x * radial + 2f * p1 * x * y + p2 * (r2 + 2f * x * x);
            float distortedY = y * radial + p1 * (r2 + 2f * y * y) + 2f * p2 * x * y;
            x = distortedX;
            y = distortedY;
        }
    }
}
