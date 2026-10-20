using System;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using StrikerPC.Control;
using StrikerPC.State;
using StrikerPC.Video;
using UnityEngine;

namespace StrikerPC.Map
{
    public sealed class DroneMapTelemetryPublisher : MonoBehaviour
    {
        [SerializeField] private VenueMapLoader mapLoader;
        [SerializeField] private Transform mapRoot;
        [SerializeField] private Transform trackedDrone;
        [Tooltip("기존 Vision 컴포넌트의 HasReceivedPose와 LatestPose를 읽는 호환 참조입니다.")]
        [SerializeField] private UnityEngine.Object trackingSource;
        [SerializeField] private DronePoseHistoryProvider poseHistory;
        [SerializeField] private TelloStateReceiver stateReceiver;
        [SerializeField] private TelloControlOrchestrator controlOrchestrator;
        [SerializeField] private PilotSharedFrameTexture sharedFrameTexture;
        [SerializeField, Range(5f, 10f)] private float publishRateHz = 10f;
        [SerializeField, Min(1)] private int gatewayEventPort = 49301;

        private UdpClient client;
        private float nextPublishTime;
        private PropertyInfo hasReceivedPoseProperty;
        private PropertyInfo latestPoseProperty;
        private PropertyInfo isTrackedProperty;
        private bool hasPublishedHeading;
        private int lastPublishedHeadingDegrees;
        private string lastPublishedHeadingMapId;
        private int lastPublishedHeadingMapVersion;
        private string lastPublishedHeadingScanId;

        // 런타임 bootstrap에서 통합 pose 수신기를 telemetry 원본으로 연결한다.
        public void ConfigureTrackingSource(UnityEngine.Object source)
        {
            trackingSource = source;
            hasReceivedPoseProperty = null;
            latestPoseProperty = null;
            isTrackedProperty = null;
        }

        // 텔레메트리 송신 소켓을 준비한다.
        private void OnEnable()
        {
            publishRateHz = 10f;
            hasPublishedHeading = false;
            lastPublishedHeadingMapId = null;
            lastPublishedHeadingScanId = null;
            stateReceiver ??= FindFirstObjectByType<TelloStateReceiver>();
            controlOrchestrator ??= FindFirstObjectByType<TelloControlOrchestrator>();
            sharedFrameTexture ??= FindFirstObjectByType<PilotSharedFrameTexture>();
            client = new UdpClient();
        }

        // 10Hz 이하로 전시장 카메라 pose와 추적 상태를 전송한다.
        private void Update()
        {
            if (Time.unscaledTime < nextPublishTime || mapLoader?.Current == null || mapRoot == null)
            {
                return;
            }

            nextPublishTime = Time.unscaledTime + 1f / publishRateHz;
            VenueMapDefinition map = mapLoader.Current;
            DronePoseSampleV2 pose = default;
            bool hasVenuePose = poseHistory != null && poseHistory.TryGetLatestPose(out pose);
            bool isTracking = hasVenuePose ? pose.IsTracked : IsTrackingValid();
            DroneNavigationStatus navigationStatus = DroneNavigationStatusResolver.Resolve(
                false,
                hasVenuePose ? pose.trackingState : DroneTrackingStateV2.Initializing);
            Vector3 venuePosition = hasVenuePose
                ? pose.venuePosition
                : trackedDrone != null
                    ? mapRoot.InverseTransformPoint(trackedDrone.position)
                    : Vector3.zero;
            Quaternion venueRotation = hasVenuePose
                ? pose.venueRotation
                : trackedDrone != null
                    ? Quaternion.Inverse(mapRoot.rotation) * trackedDrone.rotation
                    : Quaternion.identity;
            TelloConnectionState state = stateReceiver != null
                ? stateReceiver.Current
                : new TelloConnectionState();
            float bearingDegrees = AprilTagBoardBearing.FromVenueRotation(
                venueRotation,
                map.boardToVenue);

            DroneMapTelemetry telemetry = new DroneMapTelemetry
            {
                mapId = map.mapId,
                mapVersion = map.version,
                scanId = map.scanId,
                mapX = venuePosition.x,
                mapZ = venuePosition.z,
                venueY = venuePosition.y,
                yawDegrees = venueRotation.eulerAngles.y,
                bearingDegrees = bearingDegrees,
                bearingCardinal = AprilTagBoardBearing.GetCardinal(bearingDegrees),
                bearingReference = AprilTagBoardBearing.Reference,
                rotationX = venueRotation.x,
                rotationY = venueRotation.y,
                rotationZ = venueRotation.z,
                rotationW = venueRotation.w,
                tracking = isTracking,
                trackingState = hasVenuePose ? pose.trackingState.ToString() : "Initializing",
                navigationStatus = DroneNavigationStatusResolver.ToContractValue(navigationStatus),
                confidence = hasVenuePose ? pose.confidence : 0f,
                positionStdDevX = hasVenuePose ? pose.positionStdDev.x : 0f,
                positionStdDevY = hasVenuePose ? pose.positionStdDev.y : 0f,
                positionStdDevZ = hasVenuePose ? pose.positionStdDev.z : 0f,
                yawStdDev = hasVenuePose ? pose.yawStdDev : 0f,
                sourceTimestampNs = hasVenuePose ? (long)pose.sourceTimestampNs : 0L,
                stateConnected = state.stateConnected,
                batteryPercent = state.batteryPercent,
                flightTimeSeconds = state.flightTimeSeconds,
                heightMeters = state.heightCentimeters / 100f,
                pitchDegrees = state.pitch,
                rollDegrees = state.roll,
                controlPath = controlOrchestrator != null
                    ? controlOrchestrator.ActiveMode.ToString()
                    : "Disconnected",
                videoFrameAgeMilliseconds = sharedFrameTexture != null
                    ? (float)sharedFrameTexture.LastFrameAgeMilliseconds
                    : -1f,
                poseAgeMilliseconds = CalculateMonotonicAgeMilliseconds(
                    hasVenuePose ? pose.sourceTimestampNs : 0UL),
                visualAnchorCount = hasVenuePose ? pose.visualAnchorCount : 0,
                localMapReady = hasVenuePose && pose.localMapReady,
                localMapDegraded = hasVenuePose && pose.localMapDegraded,
                relocalizing = hasVenuePose && pose.relocalizing,
                timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
            };

            SendTelemetry(telemetry);
            if (ShouldPublishHeading(map, bearingDegrees, out int headingDegrees))
            {
                SendTelemetry(new DroneHeadingTelemetry
                {
                    mapId = map.mapId,
                    mapVersion = map.version,
                    scanId = map.scanId,
                    headingDegrees = headingDegrees,
                    headingReference = AprilTagBoardBearing.Reference,
                    sourceTimestampNs = hasVenuePose ? (long)pose.sourceTimestampNs : 0L,
                    timestamp = telemetry.timestamp
                });
            }
        }

        // 태그 보드 상대 방위의 표시값 또는 지도 계약이 바뀐 경우에만 송신한다.
        private bool ShouldPublishHeading(
            VenueMapDefinition map,
            float bearingDegrees,
            out int headingDegrees)
        {
            headingDegrees = Mathf.RoundToInt(AprilTagBoardBearing.Normalize(bearingDegrees)) % 360;
            if (hasPublishedHeading
                && lastPublishedHeadingDegrees == headingDegrees
                && lastPublishedHeadingMapId == map.mapId
                && lastPublishedHeadingMapVersion == map.version
                && lastPublishedHeadingScanId == map.scanId)
            {
                return false;
            }

            hasPublishedHeading = true;
            lastPublishedHeadingDegrees = headingDegrees;
            lastPublishedHeadingMapId = map.mapId;
            lastPublishedHeadingMapVersion = map.version;
            lastPublishedHeadingScanId = map.scanId;
            return true;
        }

        // 직렬화한 텔레메트리를 Gateway UDP 포트로 전송한다.
        private void SendTelemetry(object message)
        {
            byte[] payload = Encoding.UTF8.GetBytes(JsonUtility.ToJson(message));
            client.Send(payload, payload.Length, new IPEndPoint(IPAddress.Loopback, gatewayEventPort));
        }

        // Python과 공유하는 monotonic timestamp로 pose 지연 시간을 계산한다.
        private static float CalculateMonotonicAgeMilliseconds(ulong sourceTimestampNs)
        {
            if (sourceTimestampNs == 0UL)
            {
                return -1f;
            }

            return (float)global::StrikerPC.StrikerPCMonotonicClock
                .CalculateAgeMilliseconds(sourceTimestampNs);
        }

        // 비전 컴포넌트가 있으면 실제 추적 상태를 확인한다.
        private bool IsTrackingValid()
        {
            if (trackedDrone == null)
            {
                return false;
            }

            if (trackingSource == null)
            {
                return true;
            }

            hasReceivedPoseProperty ??= trackingSource.GetType().GetProperty("HasReceivedPose");
            latestPoseProperty ??= trackingSource.GetType().GetProperty("LatestPose");
            if (hasReceivedPoseProperty?.GetValue(trackingSource) is not bool hasPose || !hasPose)
            {
                return false;
            }

            object latestPose = latestPoseProperty?.GetValue(trackingSource);
            if (latestPose == null)
            {
                return false;
            }

            isTrackedProperty ??= latestPose.GetType().GetProperty("IsTracked");
            return isTrackedProperty?.GetValue(latestPose) is bool isTracked && isTracked;
        }

        // 텔레메트리 소켓을 닫는다.
        private void OnDisable()
        {
            client?.Dispose();
            client = null;
        }
    }
}
