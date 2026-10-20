using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace StrikerPC.Video
{
    [DisallowMultipleComponent]
    public sealed class MissionVideoOverlay : MonoBehaviour
    {
        private sealed class BoxView
        {
            public GameObject Root;
            public LineRenderer Line;
        }

        private const string DetectionFrameType = "mission-detection-frame";
        private const string TargetStateType = "mission-target-state";
        private const string LiveKitTransport = "livekit";
        private const string NormalizedTopLeft = "normalized-top-left";

        [SerializeField] private PilotAnnotationOverlay annotationOverlay;
        [SerializeField] private Color detectionColor = Color.green;
        [SerializeField] private Color selectedColor = Color.red;
        [SerializeField] private Color lockedColor = Color.red;
        [SerializeField, Min(0.001f)] private float detectionStrokeWidth = 0.002f;
        [SerializeField, Min(0.001f)] private float targetStrokeWidth = 0.002f;
        [SerializeField, Min(0.05f)] private float detectionTtlSeconds = 0.75f;
        [SerializeField] private float detectionDepth = -0.012f;
        [SerializeField] private float targetDepth = -0.01f;

        private readonly List<BoxView> detectionViews = new List<BoxView>();
        private readonly Stack<BoxView> detectionViewPool = new Stack<BoxView>();
        private Transform overlayRoot;
        private BoxView targetView;
        private Material lineMaterial;
        private float detectionExpiresAt = -1f;
        private string targetStreamKey;
        private long latestTargetSequence = -1;

        public int DetectionCount => detectionViews.Count;
        public bool HasActiveTarget { get; private set; }
        public bool TargetLocked { get; private set; }

        // 영상 주석과 동일한 FPV 오버레이 기준을 준비한다.
        private void Awake()
        {
            detectionColor = Color.green;
            selectedColor = Color.red;
            lockedColor = Color.red;
            EnsureOverlayRoot();
        }

        // 일시 검출 박스의 로컬 만료 시간을 확인한다.
        private void Update()
        {
            if (detectionExpiresAt >= 0f && Time.unscaledTime >= detectionExpiresAt)
            {
                ClearDetections();
            }
        }

        // 런타임에 생성한 선과 Material을 정리한다.
        private void OnDestroy()
        {
            ClearDetections();
            foreach (BoxView view in detectionViewPool)
            {
                DestroyRuntimeObject(view.Root);
            }
            detectionViewPool.Clear();

            if (targetView != null)
            {
                DestroyRuntimeObject(targetView.Root);
                targetView = null;
            }
            if (lineMaterial != null)
            {
                DestroyRuntimeObject(lineMaterial);
                lineMaterial = null;
            }
        }

        // 기존 파일럿 주석 오버레이와 같은 Transform 기준을 사용한다.
        public void Configure(PilotAnnotationOverlay source)
        {
            annotationOverlay = source;
            Transform sharedRoot = source != null ? source.OverlayRoot : transform;
            if (overlayRoot == sharedRoot)
            {
                return;
            }

            overlayRoot = sharedRoot;
            foreach (BoxView view in detectionViews)
            {
                view.Root.transform.SetParent(overlayRoot, false);
            }
            if (targetView != null)
            {
                targetView.Root.transform.SetParent(overlayRoot, false);
            }
        }

        // Gateway 임무 메시지를 검출 프레임 또는 선택 상태로 분기한다.
        public void ProcessCommand(string json)
        {
            if (string.IsNullOrWhiteSpace(json))
            {
                return;
            }

            MissionMessageHeader header = JsonUtility.FromJson<MissionMessageHeader>(json);
            if (header == null || header.schemaVersion != 1)
            {
                return;
            }

            if (header.type == DetectionFrameType)
            {
                ApplyDetectionFrame(JsonUtility.FromJson<MissionDetectionFrame>(json));
            }
            else if (header.type == TargetStateType)
            {
                ApplyTargetState(JsonUtility.FromJson<MissionTargetState>(json));
            }
        }

        // 수신한 검출 프레임의 유효한 박스를 발신자와 sequence에 관계없이 교체한다.
        public void ApplyDetectionFrame(MissionDetectionFrame frame)
        {
            if (frame == null
                || frame.schemaVersion != 1
                || frame.type != DetectionFrameType
                || frame.sourceTransport != LiveKitTransport
                || frame.coordinateSpace != NormalizedTopLeft)
            {
                return;
            }

            ClearDetections();
            MissionDetectionBox[] boxes = frame.boxes ?? Array.Empty<MissionDetectionBox>();
            foreach (MissionDetectionBox box in boxes)
            {
                if (!IsValidBox(box))
                {
                    continue;
                }

                BoxView view = GetDetectionView();
                view.Root.name = $"MissionDetection-{box.index}";
                ConfigureBox(view, box, detectionColor, detectionStrokeWidth, detectionDepth, 21);
                detectionViews.Add(view);
            }

            detectionExpiresAt = detectionViews.Count > 0
                ? Time.unscaledTime + detectionTtlSeconds
                : -1f;
        }

        // 선택 또는 잠금된 목표 박스를 빨간색 상태선으로 유지한다.
        public void ApplyTargetState(MissionTargetState state)
        {
            if (state == null
                || state.schemaVersion != 1
                || state.type != TargetStateType
                || state.sourceTransport != LiveKitTransport
                || IsOlderSequence(
                    BuildStreamKey(state.sessionId, state.droneId, state.sourceTrack),
                    state.sequence,
                    ref targetStreamKey,
                    ref latestTargetSequence))
            {
                return;
            }

            if (!state.active || string.IsNullOrWhiteSpace(state.targetId) || !IsValidBox(state.box))
            {
                HideTarget();
                return;
            }

            EnsureOverlayRoot();
            targetView ??= CreateView("MissionTarget");
            targetView.Root.transform.SetParent(overlayRoot, false);
            targetView.Root.name = $"MissionTarget-{state.targetId}";
            ConfigureBox(
                targetView,
                state.box,
                state.locked ? lockedColor : selectedColor,
                targetStrokeWidth,
                targetDepth,
                22);
            HasActiveTarget = true;
            TargetLocked = state.locked;
        }

        // 현재 일시 검출 박스를 모두 풀로 반환한다.
        public void ClearDetections()
        {
            foreach (BoxView view in detectionViews)
            {
                view.Root.SetActive(false);
                detectionViewPool.Push(view);
            }
            detectionViews.Clear();
            detectionExpiresAt = -1f;
        }

        // 선택 목표 표시를 비활성 상태로 바꾼다.
        private void HideTarget()
        {
            if (targetView != null)
            {
                targetView.Root.SetActive(false);
            }
            HasActiveTarget = false;
            TargetLocked = false;
        }

        // 영상 주석 오버레이가 없을 때만 현재 Transform을 기준으로 사용한다.
        private void EnsureOverlayRoot()
        {
            Transform desired = annotationOverlay != null ? annotationOverlay.OverlayRoot : transform;
            if (overlayRoot != desired)
            {
                Configure(annotationOverlay);
            }
        }

        // 재사용 가능한 검출 박스 View를 반환한다.
        private BoxView GetDetectionView()
        {
            EnsureOverlayRoot();
            BoxView view = detectionViewPool.Count > 0
                ? detectionViewPool.Pop()
                : CreateView("MissionDetection");
            view.Root.transform.SetParent(overlayRoot, false);
            return view;
        }

        // 정규화 top-left 박스를 FPV Quad 로컬 사각형으로 표시한다.
        private void ConfigureBox(
            BoxView view,
            MissionDetectionBox box,
            Color color,
            float width,
            float depth,
            int sortingOrder)
        {
            Vector2 topLeft = PilotAnnotationOverlay.NormalizedToLocal(box.x, box.y);
            Vector2 topRight = PilotAnnotationOverlay.NormalizedToLocal(box.x + box.width, box.y);
            Vector2 bottomRight = PilotAnnotationOverlay.NormalizedToLocal(
                box.x + box.width,
                box.y + box.height);
            Vector2 bottomLeft = PilotAnnotationOverlay.NormalizedToLocal(box.x, box.y + box.height);

            view.Root.transform.localPosition = new Vector3(0f, 0f, depth);
            view.Line.loop = true;
            view.Line.positionCount = 4;
            view.Line.SetPosition(0, new Vector3(topLeft.x, topLeft.y, 0f));
            view.Line.SetPosition(1, new Vector3(topRight.x, topRight.y, 0f));
            view.Line.SetPosition(2, new Vector3(bottomRight.x, bottomRight.y, 0f));
            view.Line.SetPosition(3, new Vector3(bottomLeft.x, bottomLeft.y, 0f));
            view.Line.startWidth = width;
            view.Line.endWidth = width;
            view.Line.startColor = color;
            view.Line.endColor = color;
            view.Line.sortingOrder = sortingOrder;
            view.Line.enabled = true;
            view.Root.SetActive(true);
        }

        // 임무 박스용 LineRenderer를 생성한다.
        private BoxView CreateView(string viewName)
        {
            GameObject root = new GameObject(viewName);
            root.transform.SetParent(overlayRoot != null ? overlayRoot : transform, false);
            LineRenderer line = root.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.alignment = LineAlignment.View;
            line.numCapVertices = 2;
            line.numCornerVertices = 2;
            line.sharedMaterial = GetLineMaterial();
            line.shadowCastingMode = ShadowCastingMode.Off;
            line.receiveShadows = false;
            return new BoxView { Root = root, Line = line };
        }

        // 상태별 LineRenderer 정점 색을 실제 화면에 반영하는 재질을 반환한다.
        private Material GetLineMaterial()
        {
            if (lineMaterial != null)
            {
                return lineMaterial;
            }

            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null)
            {
                shader = Shader.Find("Universal Render Pipeline/Unlit");
            }
            lineMaterial = new Material(shader)
            {
                name = "Mission Video Overlay Runtime Material",
                color = Color.white
            };
            if (lineMaterial.HasProperty("_BaseColor"))
            {
                lineMaterial.SetColor("_BaseColor", Color.white);
            }
            return lineMaterial;
        }

        // 세션과 영상 source가 바뀌면 sequence 비교 기준을 초기화한다.
        private static bool IsOlderSequence(
            string streamKey,
            long sequence,
            ref string activeStreamKey,
            ref long latestSequence)
        {
            if (!string.Equals(streamKey, activeStreamKey, StringComparison.Ordinal))
            {
                activeStreamKey = streamKey;
                latestSequence = sequence;
                return false;
            }
            if (sequence <= latestSequence)
            {
                return true;
            }
            latestSequence = sequence;
            return false;
        }

        // sequence 비교에 사용할 세션별 영상 source 키를 만든다.
        private static string BuildStreamKey(string sessionId, string droneId, string sourceTrack)
        {
            return $"{sessionId}\n{droneId}\n{sourceTrack}";
        }

        // 박스가 정규화 영상 범위 안의 유한한 사각형인지 확인한다.
        private static bool IsValidBox(MissionDetectionBox box)
        {
            if (box == null
                || !IsFinite(box.x)
                || !IsFinite(box.y)
                || !IsFinite(box.width)
                || !IsFinite(box.height))
            {
                return false;
            }
            return box.x >= 0f
                && box.y >= 0f
                && box.width > 0f
                && box.height > 0f
                && box.x + box.width <= 1f
                && box.y + box.height <= 1f;
        }

        // float 값이 유한한지 확인한다.
        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        // Play Mode와 Edit Mode에 맞게 런타임 오브젝트를 제거한다.
        private static void DestroyRuntimeObject(UnityEngine.Object target)
        {
            if (target == null)
            {
                return;
            }
            if (Application.isPlaying)
            {
                Destroy(target);
            }
            else
            {
                DestroyImmediate(target);
            }
        }
    }
}
