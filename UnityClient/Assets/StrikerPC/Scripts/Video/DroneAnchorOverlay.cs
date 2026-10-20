using System;
using System.Collections.Generic;
using StrikerPC.Map;
using UnityEngine;

namespace StrikerPC.Video
{
    public sealed class DroneAnchorOverlay : MonoBehaviour
    {
        private const ulong MaximumPoseDeltaNs = 50_000_000UL;
        private const float FusedPoseProjectionYawOffsetDegrees = 180f;
        private const float OverlayDepth = -0.01f;

        [SerializeField] private PilotSharedFrameTexture frameSource;
        [SerializeField] private DronePoseHistoryProvider poseHistory;
        [SerializeField] private DroneVisualPoseReceiver visualPoseSource;
        [SerializeField] private DroneCameraCalibration calibration;
        [SerializeField] private WorldPingManager pingManager;
        [SerializeField] private Transform overlayRoot;
        [SerializeField, Range(0.5f, 0.98f)] private float edgeClamp = 0.9f;
        [SerializeField] private Color markerColor = new Color(1f, 0.25f, 0.1f, 1f);
        [SerializeField] private string localPilotIdentity = "quest-pilot";
        [SerializeField] private Color localPilotColor = new Color(0.153f, 0.827f, 1f, 1f);
        [SerializeField] private Color adminColor = new Color(1f, 0.69f, 0.125f, 1f);
        [SerializeField] private Color remoteColor = new Color(1f, 0.302f, 0.427f, 1f);
        [SerializeField] private Color sensorPredictionTint = new Color(1f, 0.7f, 0.1f, 1f);
        [SerializeField, Range(0f, 1f)] private float sensorPredictionTintAmount = 0.55f;
        [SerializeField, Range(0f, 1f)] private float minimumPredictionConfidence = 0.2f;
        [SerializeField, Min(0.01f)] private float maximumPredictionPositionStdDevMeters = 0.75f;
        [SerializeField, Range(1f, 90f)] private float maximumPredictionYawStdDevDegrees = 20f;
        [SerializeField, Min(0.05f)] private float groundRingRadiusMeters = 0.25f;
        [SerializeField, Range(8, 48)] private int groundRingSegments = 20;
        [SerializeField, Min(0.1f)] private float beaconHeightMeters = 1.2f;
        [SerializeField, Range(0.08f, 0.5f)] private float maximumBeamScreenLength = 0.32f;
        [SerializeField, Range(0.02f, 0.12f)] private float beaconCapSize = 0.035f;
        [SerializeField, Range(0.04f, 0.2f)] private float edgeIndicatorSize = 0.1f;
        [SerializeField, Range(0f, 0.3f)] private float pulseAmount = 0.12f;
        [SerializeField, Min(0.1f)] private float pulseRateHz = 1f;

        private readonly Dictionary<string, PingOverlayView> views =
            new Dictionary<string, PingOverlayView>();
        private readonly Stack<PingOverlayView> viewPool = new Stack<PingOverlayView>();
        private readonly HashSet<string> visibleIds = new HashSet<string>();
        private readonly List<string> staleIds = new List<string>();
        private Material lineMaterial;
        private bool warnedResolutionMismatch;

        // 최신 융합 pose에 맞춰 지면 비콘 또는 화면 밖 방향표시를 갱신한다.
        private void LateUpdate()
        {
            if (calibration == null
                || !calibration.IsValid
                || pingManager == null
                || overlayRoot == null
                || !IsFrameResolutionCompatible())
            {
                HideAll();
                return;
            }

            if (!TryGetCameraPose(
                out DronePoseSampleV2 cameraPose,
                out bool usesSensorPrediction))
            {
                HideAll();
                return;
            }

            visibleIds.Clear();
            float pulse = 0.5f + 0.5f * Mathf.Sin(
                Time.unscaledTime * Mathf.PI * 2f * pulseRateHz);
            foreach (KeyValuePair<string, WorldPingEvent> pair in pingManager.ActiveAnchors)
            {
                Vector3 anchor = new Vector3(
                    pair.Value.venueX,
                    pair.Value.venueY,
                    pair.Value.venueZ);
                bool isProjectable = TryProjectNormalized(
                    anchor,
                    cameraPose,
                    out Vector2 projected);
                bool hasDirection = isProjectable
                    || TryGetBehindCameraDirection(
                        anchor,
                        cameraPose,
                        out projected);
                if (!hasDirection)
                {
                    HideView(pair.Key);
                    continue;
                }

                PingOverlayView view = GetOrCreateView(pair.Key);
                Color color = ResolveMarkerColor(pair.Value);
                if (usesSensorPrediction)
                {
                    color = Color.Lerp(
                        color,
                        sensorPredictionTint,
                        ResolvePredictionTintAmount(cameraPose));
                }
                if (isProjectable && IsInsideFrame(projected))
                {
                    ShowGroundBeacon(view, anchor, projected, cameraPose, color, pulse);
                }
                else
                {
                    ShowEdgeIndicator(view, projected, color, pulse);
                }
                visibleIds.Add(pair.Key);
            }

            ReleaseDeletedViews();
        }

        // 융합 지도 pose를 우선하고 시각 pose는 지도 pose가 없을 때만 사용한다.
        private bool TryGetCameraPose(
            out DronePoseSampleV2 cameraPose,
            out bool usesSensorPrediction)
        {
            if (frameSource != null
                && frameSource.isActiveAndEnabled
                && frameSource.LastSourceTimestampNs > 0UL)
            {
                ulong frameTimestampNs = frameSource.LastSourceTimestampNs;
                if (poseHistory != null
                    && (poseHistory.TryGetClosestPose(
                        frameTimestampNs,
                        MaximumPoseDeltaNs,
                        out cameraPose)
                        || poseHistory.TryGetLatestPose(out cameraPose))
                    && IsUsableFallbackPose(cameraPose))
                {
                    usesSensorPrediction = cameraPose.trackingState
                        is DroneTrackingStateV2.SensorPredicted
                        or DroneTrackingStateV2.Uncertain;
                    PreparePoseForProjection(ref cameraPose);
                    return true;
                }

                if (visualPoseSource != null
                    && visualPoseSource.TryGetClosestPose(
                        frameTimestampNs,
                        MaximumPoseDeltaNs,
                        out cameraPose))
                {
                    usesSensorPrediction = false;
                    return true;
                }
            }

            cameraPose = default;
            usesSensorPrediction = false;
            return false;
        }

        // 융합 예측값은 오차가 커도 수치가 유효하면 화면 표시에는 사용한다.
        private bool IsUsableFallbackPose(DronePoseSampleV2 pose)
        {
            if (pose.trackingState is DroneTrackingStateV2.AprilTagLocked
                or DroneTrackingStateV2.VisualTracked)
            {
                return true;
            }

            float maximumPositionStdDev = Mathf.Max(
                Mathf.Abs(pose.positionStdDev.x),
                Mathf.Abs(pose.positionStdDev.y),
                Mathf.Abs(pose.positionStdDev.z));
            return (pose.trackingState is DroneTrackingStateV2.SensorPredicted
                    or DroneTrackingStateV2.Uncertain)
                && IsFinite(pose.confidence)
                && IsFinite(maximumPositionStdDev)
                && IsFinite(pose.yawStdDev);
        }

        // 지도용 융합 방향을 FPV 카메라 전방축으로 바꾸고 Uncertain을 예측 pose로 투영한다.
        private static void PreparePoseForProjection(ref DronePoseSampleV2 pose)
        {
            pose.venueRotation *= Quaternion.Euler(
                0f,
                FusedPoseProjectionYawOffsetDegrees,
                0f);
            if (pose.trackingState == DroneTrackingStateV2.Uncertain)
            {
                pose.trackingState = DroneTrackingStateV2.SensorPredicted;
            }
        }

        // 오차가 큰 예측값은 숨기지 않고 주황색 경고 비율만 높인다.
        private float ResolvePredictionTintAmount(DronePoseSampleV2 pose)
        {
            float maximumPositionStdDev = Mathf.Max(
                Mathf.Abs(pose.positionStdDev.x),
                Mathf.Abs(pose.positionStdDev.y),
                Mathf.Abs(pose.positionStdDev.z));
            bool outsidePreferredBounds = pose.confidence < minimumPredictionConfidence
                || maximumPositionStdDev > maximumPredictionPositionStdDevMeters
                || pose.yawStdDev > maximumPredictionYawStdDevDegrees;
            return outsidePreferredBounds
                ? Mathf.Max(sensorPredictionTintAmount, 0.85f)
                : sensorPredictionTintAmount;
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        // 실제 FPV Texture와 보정 해상도가 같을 때만 투영을 허용한다.
        private bool IsFrameResolutionCompatible()
        {
            if (frameSource == null || !frameSource.isActiveAndEnabled || frameSource.Texture == null)
            {
                return false;
            }

            bool matches = frameSource.Texture.width == calibration.ImageWidth
                && frameSource.Texture.height == calibration.ImageHeight;
            if (!matches && !warnedResolutionMismatch)
            {
                warnedResolutionMismatch = true;
                Debug.LogWarning(
                    $"[FPV Overlay] 프레임 {frameSource.Texture.width}x{frameSource.Texture.height}와 "
                    + $"보정 {calibration.ImageWidth}x{calibration.ImageHeight} 해상도가 다릅니다.",
                    this);
            }
            else if (matches)
            {
                warnedResolutionMismatch = false;
            }
            return matches;
        }

        // 화면 안 핑을 원근감 있는 지면 링, 짧은 빔과 다이아몬드로 표시한다.
        private void ShowGroundBeacon(
            PingOverlayView view,
            Vector3 anchor,
            Vector2 projected,
            DronePoseSampleV2 cameraPose,
            Color color,
            float pulse)
        {
            view.Root.SetActive(true);
            view.Edge.gameObject.SetActive(false);

            float ringRadius = groundRingRadiusMeters
                * (1f + Mathf.Lerp(0f, pulseAmount, pulse));
            bool ringVisible = UpdateGroundRing(
                view,
                anchor,
                cameraPose,
                ringRadius,
                color,
                pulse);
            view.Ring.gameObject.SetActive(ringVisible);

            if (!TryProjectNormalized(
                    anchor + Vector3.up * beaconHeightMeters,
                    cameraPose,
                    out Vector2 projectedTop))
            {
                view.Beam.gameObject.SetActive(false);
                view.Cap.gameObject.SetActive(false);
                return;
            }

            Vector2 beamDelta = projectedTop - projected;
            if (beamDelta.magnitude > maximumBeamScreenLength)
            {
                projectedTop = projected
                    + beamDelta.normalized * maximumBeamScreenLength;
            }

            view.Beam.gameObject.SetActive(true);
            view.Beam.positionCount = 2;
            view.Beam.SetPosition(0, ToOverlayPoint(projected));
            view.Beam.SetPosition(1, ToOverlayPoint(projectedTop));
            Color beamBase = WithAlpha(color, 0.85f);
            Color beamTop = WithAlpha(color, 0.28f);
            view.Beam.startColor = beamBase;
            view.Beam.endColor = beamTop;

            view.Cap.gameObject.SetActive(true);
            SetDiamond(view.Cap, projectedTop, beaconCapSize, WithAlpha(color, 0.9f));
        }

        // venue 지면 원을 표본화해 FPV 화면의 원근 타원으로 투영한다.
        private bool UpdateGroundRing(
            PingOverlayView view,
            Vector3 anchor,
            DronePoseSampleV2 cameraPose,
            float radius,
            Color color,
            float pulse)
        {
            int segments = Mathf.Clamp(groundRingSegments, 8, 48);
            view.EnsureRingBuffer(segments + 1);
            for (int index = 0; index <= segments; index++)
            {
                float angle = Mathf.PI * 2f * index / segments;
                Vector3 venuePoint = anchor + new Vector3(
                    Mathf.Cos(angle) * radius,
                    0.015f,
                    Mathf.Sin(angle) * radius);
                if (!TryProjectNormalized(venuePoint, cameraPose, out Vector2 point))
                {
                    return false;
                }
                view.RingPositions[index] = ToOverlayPoint(point);
            }

            view.Ring.positionCount = segments + 1;
            view.Ring.SetPositions(view.RingPositions);
            Color ringColor = WithAlpha(color, Mathf.Lerp(0.45f, 0.9f, pulse));
            view.Ring.startColor = ringColor;
            view.Ring.endColor = ringColor;
            return true;
        }

        // 화면 밖 핑은 해당 바깥 방향을 향하는 꺾쇠로 표시한다.
        private void ShowEdgeIndicator(
            PingOverlayView view,
            Vector2 target,
            Color color,
            float pulse)
        {
            view.Root.SetActive(true);
            view.Ring.gameObject.SetActive(false);
            view.Beam.gameObject.SetActive(false);
            view.Cap.gameObject.SetActive(false);
            view.Edge.gameObject.SetActive(true);

            Vector2 tip = ClampToEdge(target, Mathf.Clamp(edgeClamp, 0.5f, 0.98f));
            Vector2 direction = GetEdgeDirection(target, tip);
            Vector2 back = tip - direction * edgeIndicatorSize;
            Vector2 perpendicular = new Vector2(-direction.y, direction.x)
                * edgeIndicatorSize * 0.55f;
            view.Edge.positionCount = 3;
            view.Edge.SetPosition(0, ToOverlayPoint(back + perpendicular));
            view.Edge.SetPosition(1, ToOverlayPoint(tip));
            view.Edge.SetPosition(2, ToOverlayPoint(back - perpendicular));
            Color edgeColor = WithAlpha(color, Mathf.Lerp(0.6f, 1f, pulse));
            view.Edge.startColor = edgeColor;
            view.Edge.endColor = edgeColor;
        }

        // Venue 점을 FPV 정규화 좌표로 투영한다.
        private bool TryProjectNormalized(
            Vector3 venuePoint,
            DronePoseSampleV2 cameraPose,
            out Vector2 normalized)
        {
            normalized = default;
            if (!DroneCameraProjector.TryProject(
                    venuePoint,
                    cameraPose,
                    calibration,
                    out Vector2 pixel,
                    out _))
            {
                return false;
            }

            normalized = new Vector2(
                pixel.x / calibration.ImageWidth * 2f - 1f,
                1f - pixel.y / calibration.ImageHeight * 2f);
            return true;
        }

        // 카메라 뒤로 넘어간 핑도 화면 평면 방향이 명확하면 경계 화살표를 유지한다.
        private static bool TryGetBehindCameraDirection(
            Vector3 venuePoint,
            DronePoseSampleV2 cameraPose,
            out Vector2 direction)
        {
            direction = default;
            if (!cameraPose.IsTracked)
            {
                return false;
            }

            Vector3 cameraPoint = Quaternion.Inverse(cameraPose.venueRotation)
                * (venuePoint - cameraPose.venuePosition);
            if (cameraPoint.z > 0.001f)
            {
                return false;
            }

            Vector2 screenDirection = new Vector2(-cameraPoint.x, cameraPoint.y);
            if (screenDirection.sqrMagnitude < 0.000001f)
            {
                return false;
            }

            direction = screenDirection.normalized * 2f;
            return true;
        }

        // 정규화 좌표가 FPV 화면 안에 있는지 확인한다.
        private static bool IsInsideFrame(Vector2 point)
        {
            return Mathf.Abs(point.x) <= 1f && Mathf.Abs(point.y) <= 1f;
        }

        // 화면 중심에서 대상까지의 광선을 직사각형 경계에 맞춘다.
        private static Vector2 ClampToEdge(Vector2 target, float limit)
        {
            float denominator = Mathf.Max(Mathf.Abs(target.x), Mathf.Abs(target.y));
            return denominator > 0.000001f
                ? target * (limit / denominator)
                : Vector2.zero;
        }

        // 경계 꺾쇠가 화면 밖 실제 핑 방향을 향하도록 방향을 계산한다.
        private static Vector2 GetEdgeDirection(Vector2 target, Vector2 tip)
        {
            Vector2 direction = (target - tip).normalized;
            return direction.sqrMagnitude >= 0.000001f
                ? direction
                : target.normalized;
        }

        // 정규화 좌표를 FPV Quad의 로컬 좌표로 바꾼다.
        private static Vector3 ToOverlayPoint(Vector2 normalized)
        {
            return new Vector3(normalized.x * 0.5f, normalized.y * 0.5f, OverlayDepth);
        }

        // 빔 끝에 화면 크기가 일정한 다이아몬드를 그린다.
        private static void SetDiamond(
            LineRenderer line,
            Vector2 center,
            float size,
            Color color)
        {
            line.positionCount = 5;
            line.SetPosition(0, ToOverlayPoint(center + Vector2.up * size));
            line.SetPosition(1, ToOverlayPoint(center + Vector2.right * size));
            line.SetPosition(2, ToOverlayPoint(center + Vector2.down * size));
            line.SetPosition(3, ToOverlayPoint(center + Vector2.left * size));
            line.SetPosition(4, ToOverlayPoint(center + Vector2.up * size));
            line.startColor = color;
            line.endColor = color;
        }

        // 생성 주체에 따라 기존 지도 핑과 동일한 색상을 선택한다.
        private Color ResolveMarkerColor(WorldPingEvent pingEvent)
        {
            if (pingEvent.requesterIdentity == localPilotIdentity)
            {
                return localPilotColor;
            }
            if (pingEvent.requesterRole == "admin")
            {
                return adminColor;
            }
            if (pingEvent.requesterRole == "pilot")
            {
                return remoteColor;
            }
            return markerColor;
        }

        // 지정한 알파를 원래 색상 알파에 곱한다.
        private static Color WithAlpha(Color color, float alpha)
        {
            color.a *= alpha;
            return color;
        }

        // 핑별 렌더러 묶음을 풀에서 가져오거나 새로 만든다.
        private PingOverlayView GetOrCreateView(string id)
        {
            if (views.TryGetValue(id, out PingOverlayView existing))
            {
                return existing;
            }

            PingOverlayView view = viewPool.Count > 0
                ? viewPool.Pop()
                : CreateView();
            view.Root.name = $"FpvPing-{id}";
            view.Root.SetActive(true);
            views.Add(id, view);
            return view;
        }

        // 핑 비콘과 화면 밖 방향표시를 구성하는 LineRenderer를 생성한다.
        private PingOverlayView CreateView()
        {
            GameObject root = new GameObject("FpvPing");
            root.transform.SetParent(overlayRoot, false);
            return new PingOverlayView(
                root,
                CreateLine(root.transform, "GroundRing", 0.006f, 0.006f),
                CreateLine(root.transform, "Beam", 0.009f, 0.004f),
                CreateLine(root.transform, "BeaconCap", 0.006f, 0.006f),
                CreateLine(root.transform, "EdgeIndicator", 0.01f, 0.006f));
        }

        // 공용 Unlit 재질을 사용하는 선 렌더러를 만든다.
        private LineRenderer CreateLine(
            Transform parent,
            string name,
            float startWidth,
            float endWidth)
        {
            GameObject target = new GameObject(name);
            target.transform.SetParent(parent, false);
            LineRenderer line = target.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.alignment = LineAlignment.View;
            line.textureMode = LineTextureMode.Stretch;
            line.numCornerVertices = 2;
            line.numCapVertices = 2;
            line.startWidth = startWidth;
            line.endWidth = endWidth;
            line.sharedMaterial = GetLineMaterial();
            return line;
        }

        // 모든 FPV 핑 선이 공유할 Sprites/Default 재질을 준비한다.
        private Material GetLineMaterial()
        {
            if (lineMaterial == null)
            {
                lineMaterial = new Material(Shader.Find("Sprites/Default"))
                {
                    name = "FpvPingOverlayMaterial",
                    hideFlags = HideFlags.HideAndDontSave
                };
            }
            return lineMaterial;
        }

        // 삭제된 ID의 뷰를 비활성화해 풀로 반환한다.
        private void ReleaseDeletedViews()
        {
            staleIds.Clear();
            foreach (KeyValuePair<string, PingOverlayView> pair in views)
            {
                if (!pingManager.ActiveAnchors.ContainsKey(pair.Key))
                {
                    staleIds.Add(pair.Key);
                }
                else if (!visibleIds.Contains(pair.Key))
                {
                    pair.Value.Root.SetActive(false);
                }
            }

            foreach (string id in staleIds)
            {
                PingOverlayView view = views[id];
                views.Remove(id);
                view.Root.SetActive(false);
                viewPool.Push(view);
            }
        }

        // 한 핑의 화면 표시를 숨긴다.
        private void HideView(string id)
        {
            if (views.TryGetValue(id, out PingOverlayView view))
            {
                view.Root.SetActive(false);
            }
        }

        // 모든 FPV 핑 표시를 숨긴다.
        private void HideAll()
        {
            foreach (PingOverlayView view in views.Values)
            {
                view.Root.SetActive(false);
            }
        }

        // 컴포넌트 비활성화 시 마지막 프레임의 비콘이 남지 않게 한다.
        private void OnDisable()
        {
            warnedResolutionMismatch = false;
            HideAll();
        }

        // 런타임에 만든 공용 재질을 정리한다.
        private void OnDestroy()
        {
            if (lineMaterial == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(lineMaterial);
            }
            else
            {
                DestroyImmediate(lineMaterial);
            }
        }

        private sealed class PingOverlayView
        {
            public PingOverlayView(
                GameObject root,
                LineRenderer ring,
                LineRenderer beam,
                LineRenderer cap,
                LineRenderer edge)
            {
                Root = root;
                Ring = ring;
                Beam = beam;
                Cap = cap;
                Edge = edge;
            }

            public GameObject Root { get; }
            public LineRenderer Ring { get; }
            public LineRenderer Beam { get; }
            public LineRenderer Cap { get; }
            public LineRenderer Edge { get; }
            public Vector3[] RingPositions { get; private set; } = Array.Empty<Vector3>();

            // 링 분할 수 변경 시에만 위치 버퍼를 다시 만든다.
            public void EnsureRingBuffer(int count)
            {
                if (RingPositions.Length != count)
                {
                    RingPositions = new Vector3[count];
                }
            }
        }
    }
}
