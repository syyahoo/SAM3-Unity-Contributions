using System;
using System.Reflection;
using StrikerPC.Map;
using UnityEngine;

namespace StrikerPC.Video
{
    public sealed class DronePoseHistoryProvider : MonoBehaviour
    {
        [SerializeField] private UnityEngine.Object poseSource;
        [SerializeField] private VenueMapLoader mapLoader;
        [SerializeField, Range(16, 256)] private int capacity = 120;

        private DronePoseSampleV2[] samples;
        private int count;
        private int writeIndex;
        private ulong lastFrameId;
        private DronePoseSampleV2 latestSample;
        private PropertyInfo hasReceivedPoseProperty;
        private PropertyInfo latestPoseProperty;
        private FieldInfo frameIdField;
        private FieldInfo timestampField;
        private FieldInfo stateField;
        private FieldInfo positionField;
        private FieldInfo rotationField;
        private FieldInfo confidenceField;
        private FieldInfo positionStdDevField;
        private FieldInfo yawStdDevField;
        private FieldInfo visualAnchorCountField;
        private FieldInfo localMapReadyField;
        private FieldInfo localMapDegradedField;
        private FieldInfo relocalizingField;
        private FieldInfo sessionMapField;

        // 실제 수신 source가 아직 유효한 pose를 보유하는지 반환한다.
        public bool SourcePoseFresh
        {
            get
            {
                if (poseSource == null)
                {
                    return false;
                }

                hasReceivedPoseProperty ??= poseSource.GetType().GetProperty("HasReceivedPose");
                return hasReceivedPoseProperty?.GetValue(poseSource) is bool hasPose && hasPose;
            }
        }

        // 런타임 bootstrap에서 통합 pose 수신기로 교체한다.
        public void ConfigurePoseSource(UnityEngine.Object source)
        {
            poseSource = source;
            hasReceivedPoseProperty = null;
            latestPoseProperty = null;
            frameIdField = null;
            timestampField = null;
            stateField = null;
            positionField = null;
            rotationField = null;
            confidenceField = null;
            positionStdDevField = null;
            yawStdDevField = null;
            visualAnchorCountField = null;
            localMapReadyField = null;
            localMapDegradedField = null;
            relocalizingField = null;
            sessionMapField = null;
        }

        // Pose ring buffer를 초기화한다.
        private void Awake()
        {
            samples = new DronePoseSampleV2[Mathf.Max(16, capacity)];
        }

        // 비전 수신기의 최신 pose를 전시장 좌표로 변환해 보관한다.
        private void Update()
        {
            if (!TryReadLatestPose(out DronePoseSampleV2 sample))
            {
                return;
            }

            if (sample.frameId <= lastFrameId)
            {
                if (!IsNewPoseSession(sample, latestSample, lastFrameId))
                {
                    return;
                }
                ResetHistoryForNewSession();
            }

            lastFrameId = sample.frameId;
            latestSample = sample;
            samples[writeIndex] = sample;
            writeIndex = (writeIndex + 1) % samples.Length;
            count = Mathf.Min(count + 1, samples.Length);
        }

        // 새 Position Sync 프로세스가 sequence를 1부터 다시 시작했는지 판별한다.
        private static bool IsNewPoseSession(
            DronePoseSampleV2 candidate,
            DronePoseSampleV2 previous,
            ulong previousFrameId)
        {
            return candidate.frameId > 0UL
                && candidate.frameId <= previousFrameId
                && candidate.sourceTimestampNs > previous.sourceTimestampNs;
        }

        // 이전 프로세스의 frame ID와 보간 표본을 제거한다.
        private void ResetHistoryForNewSession()
        {
            Array.Clear(samples, 0, samples.Length);
            count = 0;
            writeIndex = 0;
            lastFrameId = 0UL;
            latestSample = default;
        }

        // 가장 최근에 수신한 전시장 pose를 반환한다.
        public bool TryGetLatestPose(out DronePoseSampleV2 sample)
        {
            sample = latestSample;
            if (count == 0)
            {
                return false;
            }

            if (!SourcePoseFresh)
            {
                sample.trackingState = DroneTrackingStateV2.Uncertain;
            }
            return true;
        }

        // 프레임 시각과 가장 가까운 pose를 50ms 제한으로 선택한다.
        public bool TryGetClosestPose(ulong sourceTimestampNs, ulong maxDeltaNs, out DronePoseSampleV2 sample)
        {
            sample = default;
            if (sourceTimestampNs == 0UL || count == 0)
            {
                return false;
            }

            ulong bestDelta = ulong.MaxValue;
            for (int offset = 0; offset < count; offset++)
            {
                int index = (writeIndex - 1 - offset + samples.Length) % samples.Length;
                DronePoseSampleV2 candidate = samples[index];
                ulong delta = candidate.sourceTimestampNs > sourceTimestampNs
                    ? candidate.sourceTimestampNs - sourceTimestampNs
                    : sourceTimestampNs - candidate.sourceTimestampNs;
                if (delta < bestDelta)
                {
                    bestDelta = delta;
                    sample = candidate;
                }
            }

            return bestDelta <= maxDeltaNs
                && sample.trackingState != DroneTrackingStateV2.Initializing;
        }

        // 기존 비전 컴포넌트를 Reflection으로 읽어 어셈블리 의존성을 피한다.
        private bool TryReadLatestPose(out DronePoseSampleV2 sample)
        {
            sample = default;
            if (poseSource == null || mapLoader?.Current == null)
            {
                return false;
            }

            Type sourceType = poseSource.GetType();
            hasReceivedPoseProperty ??= sourceType.GetProperty("HasReceivedPose");
            latestPoseProperty ??= sourceType.GetProperty("LatestPose");
            if (!SourcePoseFresh)
            {
                return false;
            }

            object boxedPose = latestPoseProperty?.GetValue(poseSource);
            if (boxedPose == null)
            {
                return false;
            }

            Type poseType = boxedPose.GetType();
            frameIdField ??= poseType.GetField("frameId");
            timestampField ??= poseType.GetField("timestamp");
            stateField ??= poseType.GetField("state");
            positionField ??= poseType.GetField("position");
            rotationField ??= poseType.GetField("rotation");
            confidenceField ??= poseType.GetField("confidence");
            positionStdDevField ??= poseType.GetField("positionStdDev");
            yawStdDevField ??= poseType.GetField("yawStdDev");
            visualAnchorCountField ??= poseType.GetField("visualAnchorCount");
            localMapReadyField ??= poseType.GetField("localMapReady");
            localMapDegradedField ??= poseType.GetField("localMapDegraded");
            relocalizingField ??= poseType.GetField("relocalizing");
            sessionMapField ??= poseType.GetField("sessionMap");
            if (frameIdField?.GetValue(boxedPose) is not ulong frameId
                || timestampField?.GetValue(boxedPose) is not double timestamp
                || positionField?.GetValue(boxedPose) is not Vector3 boardPosition
                || rotationField?.GetValue(boxedPose) is not Quaternion boardRotation)
            {
                return false;
            }

            Matrix4x4 boardToVenue = boxedPose is ContinuousDronePoseData
                ? Matrix4x4.identity
                : ReadBoardToVenue(mapLoader.Current.boardToVenue);
            sample = new DronePoseSampleV2
            {
                frameId = frameId,
                sourceTimestampNs = timestamp > 0d ? (ulong)Math.Round(timestamp * 1_000_000_000d) : 0UL,
                venuePosition = boardToVenue.MultiplyPoint3x4(boardPosition),
                venueRotation = boardToVenue.rotation * boardRotation,
                trackingState = (DroneTrackingStateV2)Convert.ToByte(stateField?.GetValue(boxedPose) ?? 3),
                confidence = confidenceField?.GetValue(boxedPose) is float confidence ? confidence : 0f,
                positionStdDev = positionStdDevField?.GetValue(boxedPose) is Vector3 stdDev
                    ? stdDev
                    : Vector3.zero,
                yawStdDev = yawStdDevField?.GetValue(boxedPose) is float yawStdDev
                    ? yawStdDev
                    : 0f,
                visualAnchorCount = visualAnchorCountField?.GetValue(boxedPose) is int anchorCount
                    ? anchorCount
                    : 0,
                localMapReady = localMapReadyField?.GetValue(boxedPose) is bool mapReady
                    && mapReady,
                localMapDegraded = localMapDegradedField?.GetValue(boxedPose) is bool mapDegraded
                    && mapDegraded,
                relocalizing = relocalizingField?.GetValue(boxedPose) is bool isRelocalizing
                    && isRelocalizing,
                sessionMap = sessionMapField?.GetValue(boxedPose) is bool isSessionMap
                    && isSessionMap
            };
            return true;
        }

        // JSON의 행 우선 boardToVenue 배열을 Unity 행렬로 변환한다.
        private static Matrix4x4 ReadBoardToVenue(float[] values)
        {
            Matrix4x4 matrix = Matrix4x4.identity;
            if (values == null || values.Length != 16)
            {
                return matrix;
            }

            for (int row = 0; row < 4; row++)
            {
                for (int column = 0; column < 4; column++)
                {
                    matrix[row, column] = values[row * 4 + column];
                }
            }
            return matrix;
        }
    }
}
