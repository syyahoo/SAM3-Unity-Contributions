using System;
using UnityEngine;

namespace StrikerPC.Video
{
    public enum DroneTrackingStateV2 : byte
    {
        Initializing = 0,
        AprilTagLocked = 1,
        VisualTracked = 2,
        SensorPredicted = 3,
        Uncertain = 4
    }

    public enum DroneNavigationStatus : byte
    {
        GpsActive = 0,
        InertialTracking = 1,
        NoSignal = 2
    }

    public static class DroneNavigationStatusResolver
    {
        // GPS 전역 pose를 우선하고, 그 외에는 신뢰 가능한 로컬 pose만 추적 중으로 표시한다.
        public static DroneNavigationStatus Resolve(
            bool gpsPoseUsable,
            DroneTrackingStateV2 trackingState)
        {
            if (gpsPoseUsable)
            {
                return DroneNavigationStatus.GpsActive;
            }

            return trackingState is DroneTrackingStateV2.AprilTagLocked
                    or DroneTrackingStateV2.VisualTracked
                    or DroneTrackingStateV2.SensorPredicted
                ? DroneNavigationStatus.InertialTracking
                : DroneNavigationStatus.NoSignal;
        }

        // 원격 JSON에서 언어와 무관하게 사용할 안정적인 상태값을 반환한다.
        public static string ToContractValue(DroneNavigationStatus status)
        {
            return status switch
            {
                DroneNavigationStatus.GpsActive => "gps-active",
                DroneNavigationStatus.InertialTracking => "inertial-tracking",
                _ => "no-signal"
            };
        }

        // Quest HUD에 표시할 한글 상태명을 반환한다.
        public static string ToDisplayLabel(DroneNavigationStatus status)
        {
            return status switch
            {
                DroneNavigationStatus.GpsActive => "GPS 활성화",
                DroneNavigationStatus.InertialTracking => "관성센서 트래킹",
                _ => "신호 없음"
            };
        }
    }

    [Serializable]
    public struct DronePoseSampleV2
    {
        public ulong frameId;
        public ulong sourceTimestampNs;
        public Vector3 venuePosition;
        public Quaternion venueRotation;
        public DroneTrackingStateV2 trackingState;
        public float confidence;
        public Vector3 positionStdDev;
        public float yawStdDev;
        public int visualAnchorCount;
        public bool localMapReady;
        public bool localMapDegraded;
        public bool relocalizing;
        public bool sessionMap;

        public bool IsTracked => trackingState is DroneTrackingStateV2.AprilTagLocked
            or DroneTrackingStateV2.VisualTracked
            or DroneTrackingStateV2.SensorPredicted;
    }
}
