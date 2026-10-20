using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;

namespace StrikerPC.Video
{
    [Serializable]
    public sealed class PilotAnnotationPoint
    {
        public float x;
        public float y;
    }

    [Serializable]
    public sealed class PilotAnnotation
    {
        public string id;
        public string type;
        public float x;
        public float y;
        public PilotAnnotationPoint start;
        public PilotAnnotationPoint end;
        public string creator;
        public long timestamp;
    }

    [Serializable]
    public sealed class PilotAnnotationCommand
    {
        public string type;
        public string id;
        public int revision;
        public string snapshotId;
        public int chunkIndex;
        public int chunkCount;
        public PilotAnnotation annotation;
        public PilotAnnotation[] annotations;
    }

    [DisallowMultipleComponent]
    public sealed class PilotAnnotationOverlay : MonoBehaviour
    {
        private sealed class AnnotationView
        {
            public GameObject Root;
            public LineRenderer Primary;
            public LineRenderer Horizontal;
            public LineRenderer Vertical;
        }

        private const string TargetType = "target";
        private const string StrokeType = "stroke";
        private const int TargetCircleSegments = 24;

        [SerializeField] private Transform overlayRoot;
        [SerializeField] private InputActionProperty toggleVisibilityAction;
        [SerializeField] private Color annotationColor = Color.white;
        [SerializeField, Min(0.001f)] private float strokeWidth = 0.002f;
        [SerializeField, Min(0.005f)] private float targetRadius = 0.035f;
        [SerializeField] private float overlayDepth = -0.015f;
        [SerializeField, Range(1, 250)] private int maxAnnotations = 250;
        [SerializeField] private UnityEvent<bool> visibilityChanged;

        private readonly Dictionary<string, PilotAnnotation> annotations =
            new Dictionary<string, PilotAnnotation>();
        private readonly Dictionary<string, AnnotationView> views =
            new Dictionary<string, AnnotationView>();
        private readonly Dictionary<string, LinkedListNode<string>> orderNodes =
            new Dictionary<string, LinkedListNode<string>>();
        private readonly LinkedList<string> annotationOrder = new LinkedList<string>();
        private readonly Stack<AnnotationView> viewPool = new Stack<AnnotationView>();

        private Material lineMaterial;
        private bool annotationsVisible = true;
        private bool focusSuppressed;
        private bool toggleEnabledByThis;
        private bool ownsToggleAction;
        private string activeSnapshotId;
        private int activeSnapshotRevision = -1;
        private int lastAppliedRevision = -1;
        private PilotAnnotation[][] snapshotChunks;
        private bool[] snapshotChunkReceived;
        private int receivedSnapshotChunks;

        public event Action<bool> EffectiveVisibilityChanged;

        public bool AnnotationsVisible => annotationsVisible;
        public bool IsEffectivelyVisible => annotationsVisible && !focusSuppressed;
        public bool HasReceivedSnapshot { get; private set; }
        public int AnnotationCount => annotations.Count;
        public Transform OverlayRoot
        {
            get
            {
                EnsureInitialized();
                return overlayRoot;
            }
        }

        // 기본 오버레이 기준과 토글 입력을 준비한다.
        private void Awake()
        {
            annotationColor = Color.white;
            EnsureInitialized();
        }

        // 오른손 A 버튼 토글 입력을 활성화한다.
        private void OnEnable()
        {
            EnsureInitialized();
            if (toggleVisibilityAction.action != null)
            {
                toggleVisibilityAction.action.performed += OnTogglePerformed;
                if (!toggleVisibilityAction.action.enabled)
                {
                    toggleVisibilityAction.action.Enable();
                    toggleEnabledByThis = true;
                }
            }

            ApplyVisibility();
        }

        // 이 컴포넌트가 연결한 토글 입력만 해제한다.
        private void OnDisable()
        {
            if (toggleVisibilityAction.action != null)
            {
                toggleVisibilityAction.action.performed -= OnTogglePerformed;
                if (toggleEnabledByThis)
                {
                    toggleVisibilityAction.action.Disable();
                    toggleEnabledByThis = false;
                }
            }
        }

        // 런타임 생성 리소스를 정리한다.
        private void OnDestroy()
        {
            foreach (AnnotationView view in views.Values)
            {
                DestroyRuntimeObject(view.Root);
            }

            while (viewPool.Count > 0)
            {
                DestroyRuntimeObject(viewPool.Pop().Root);
            }

            views.Clear();
            viewPool.Clear();
            if (lineMaterial != null)
            {
                DestroyRuntimeObject(lineMaterial);
                lineMaterial = null;
            }

            if (ownsToggleAction)
            {
                toggleVisibilityAction.action?.Dispose();
            }
        }

        // Gateway의 영상 주석 명령을 현재 상태에 반영한다.
        public void ProcessCommand(string json)
        {
            EnsureInitialized();
            if (string.IsNullOrWhiteSpace(json))
            {
                return;
            }

            PilotAnnotationCommand command = JsonUtility.FromJson<PilotAnnotationCommand>(json);
            if (command == null || string.IsNullOrWhiteSpace(command.type))
            {
                return;
            }

            if (command.type == "annotation-event")
            {
                if (command.revision < lastAppliedRevision || !IsValid(command.annotation))
                {
                    return;
                }

                CancelSnapshotAssembly();
                ApplyAnnotation(command.annotation);
                lastAppliedRevision = command.revision;
            }
            else if (command.type == "annotation-delete")
            {
                if (command.revision < lastAppliedRevision
                    || string.IsNullOrWhiteSpace(command.id))
                {
                    return;
                }

                CancelSnapshotAssembly();
                RemoveAnnotation(command.id);
                lastAppliedRevision = command.revision;
            }
            else if (command.type == "annotation-clear")
            {
                if (command.revision < lastAppliedRevision)
                {
                    return;
                }

                CancelSnapshotAssembly();
                ClearAnnotations();
                lastAppliedRevision = command.revision;
                HasReceivedSnapshot = true;
            }
            else if (command.type == "annotation-snapshot")
            {
                ApplySnapshotChunk(command);
            }
        }

        // 단일 영상 주석을 추가하거나 같은 ID의 표시를 갱신한다.
        public void ApplyAnnotation(PilotAnnotation annotation)
        {
            EnsureInitialized();
            if (!IsValid(annotation))
            {
                return;
            }

            bool exists = annotations.ContainsKey(annotation.id);
            annotations[annotation.id] = annotation;
            if (exists)
            {
                MoveToNewest(annotation.id);
            }
            else
            {
                while (annotations.Count > Mathf.Max(1, maxAnnotations))
                {
                    RemoveOldest();
                }

                LinkedListNode<string> node = annotationOrder.AddLast(annotation.id);
                orderNodes[annotation.id] = node;
            }

            AnnotationView view = GetOrCreateView(annotation.id);
            ConfigureView(view, annotation);
            view.Root.SetActive(IsEffectivelyVisible);
        }

        // 모든 영상 주석 표시와 중앙 상태를 비운다.
        public void ClearAnnotations()
        {
            foreach (AnnotationView view in views.Values)
            {
                ReturnView(view);
            }

            annotations.Clear();
            views.Clear();
            orderNodes.Clear();
            annotationOrder.Clear();
        }

        // 파일럿 영상의 주석·타겟·YOLO 추가정보 표시 상태를 전환한다.
        public void ToggleAnnotations()
        {
            if (focusSuppressed)
            {
                return;
            }
            SetAnnotationsVisible(!annotationsVisible);
        }

        // 데이터와 사용자 설정은 유지한 채 포커스 모드 동안만 표시를 억제한다.
        public void SetFocusSuppressed(bool suppressed)
        {
            if (focusSuppressed == suppressed)
            {
                return;
            }
            focusSuppressed = suppressed;
            ApplyVisibility();
            EffectiveVisibilityChanged?.Invoke(IsEffectivelyVisible);
        }

        // 영상 주석 데이터는 유지하고 렌더링 표시만 변경한다.
        public void SetAnnotationsVisible(bool visible)
        {
            EnsureInitialized();
            if (annotationsVisible == visible)
            {
                ApplyVisibility();
                return;
            }

            annotationsVisible = visible;
            ApplyVisibility();
            visibilityChanged?.Invoke(annotationsVisible);
            EffectiveVisibilityChanged?.Invoke(IsEffectivelyVisible);
        }

        // WebViewer 정규화 좌표를 FPV Quad 로컬 좌표로 변환한다.
        public static Vector2 NormalizedToLocal(float x, float y)
        {
            return new Vector2(x - 0.5f, 0.5f - y);
        }

        // 오른손 A 버튼 입력으로 공용 추가정보 부모를 토글한다.
        private void OnTogglePerformed(InputAction.CallbackContext context)
        {
            ToggleAnnotations();
        }

        // 공개 메서드 호출 전에도 오버레이 기준과 입력을 안전하게 준비한다.
        private void EnsureInitialized()
        {
            if (overlayRoot == null)
            {
                overlayRoot = transform;
            }

            EnsureToggleAction();
        }

        // Inspector 입력이 비어 있으면 Quest 오른손 A 버튼을 사용한다.
        private void EnsureToggleAction()
        {
            if (toggleVisibilityAction.action != null
                && toggleVisibilityAction.action.bindings.Count > 0)
            {
                return;
            }

            toggleVisibilityAction = new InputActionProperty(
                new InputAction(
                    "Pilot Annotation Toggle",
                    InputActionType.Button,
                    "<XRController>{RightHand}/primaryButton"));
            ownsToggleAction = true;
        }

        // 스냅샷 청크가 모두 도착하면 주석 상태를 한 번에 교체한다.
        private void ApplySnapshotChunk(PilotAnnotationCommand command)
        {
            if (string.IsNullOrWhiteSpace(command.snapshotId)
                || command.chunkCount <= 0
                || command.chunkIndex < 0
                || command.chunkIndex >= command.chunkCount
                || command.revision < lastAppliedRevision)
            {
                return;
            }

            if (activeSnapshotId != command.snapshotId)
            {
                activeSnapshotId = command.snapshotId;
                activeSnapshotRevision = command.revision;
                snapshotChunks = new PilotAnnotation[command.chunkCount][];
                snapshotChunkReceived = new bool[command.chunkCount];
                receivedSnapshotChunks = 0;
            }

            if (snapshotChunks == null
                || snapshotChunks.Length != command.chunkCount
                || activeSnapshotRevision != command.revision
                || snapshotChunkReceived[command.chunkIndex])
            {
                return;
            }

            snapshotChunks[command.chunkIndex] = command.annotations ?? Array.Empty<PilotAnnotation>();
            snapshotChunkReceived[command.chunkIndex] = true;
            receivedSnapshotChunks++;
            if (receivedSnapshotChunks != snapshotChunks.Length)
            {
                return;
            }

            List<PilotAnnotation> snapshot = new List<PilotAnnotation>();
            foreach (PilotAnnotation[] chunk in snapshotChunks)
            {
                snapshot.AddRange(chunk);
            }

            if (activeSnapshotRevision < lastAppliedRevision)
            {
                CancelSnapshotAssembly();
                return;
            }

            ReplaceAnnotations(snapshot);
            lastAppliedRevision = activeSnapshotRevision;
            HasReceivedSnapshot = true;
            CancelSnapshotAssembly();
        }

        // 진행 중인 스냅샷 조립 상태를 폐기한다.
        private void CancelSnapshotAssembly()
        {
            activeSnapshotId = null;
            activeSnapshotRevision = -1;
            snapshotChunks = null;
            snapshotChunkReceived = null;
            receivedSnapshotChunks = 0;
        }

        // 완성된 스냅샷으로 현재 주석 상태를 교체한다.
        private void ReplaceAnnotations(List<PilotAnnotation> snapshot)
        {
            ClearAnnotations();
            int start = Mathf.Max(0, snapshot.Count - Mathf.Max(1, maxAnnotations));
            for (int index = start; index < snapshot.Count; index++)
            {
                ApplyAnnotation(snapshot[index]);
            }
        }

        // 주석 종류에 맞는 선 표시를 구성한다.
        private void ConfigureView(AnnotationView view, PilotAnnotation annotation)
        {
            view.Root.name = $"PilotAnnotation-{annotation.id}";
            if (annotation.type == TargetType)
            {
                ConfigureTarget(view, annotation);
            }
            else
            {
                ConfigureStroke(view, annotation);
            }
        }

        // 목표 지점에 원형 표식과 십자선을 표시한다.
        private void ConfigureTarget(AnnotationView view, PilotAnnotation annotation)
        {
            Vector2 center = NormalizedToLocal(annotation.x, annotation.y);
            view.Root.transform.localPosition = new Vector3(center.x, center.y, overlayDepth);
            float horizontalRadius = targetRadius * HorizontalScaleCompensation();

            view.Primary.enabled = true;
            view.Primary.loop = true;
            view.Primary.positionCount = TargetCircleSegments;
            for (int index = 0; index < TargetCircleSegments; index++)
            {
                float angle = Mathf.PI * 2f * index / TargetCircleSegments;
                view.Primary.SetPosition(
                    index,
                    new Vector3(
                        Mathf.Cos(angle) * horizontalRadius,
                        Mathf.Sin(angle) * targetRadius,
                        0f));
            }

            view.Horizontal.enabled = true;
            view.Horizontal.loop = false;
            view.Horizontal.positionCount = 2;
            view.Horizontal.SetPosition(0, new Vector3(-horizontalRadius * 1.5f, 0f, 0f));
            view.Horizontal.SetPosition(1, new Vector3(horizontalRadius * 1.5f, 0f, 0f));

            view.Vertical.enabled = true;
            view.Vertical.loop = false;
            view.Vertical.positionCount = 2;
            view.Vertical.SetPosition(0, new Vector3(0f, -targetRadius * 1.5f, 0f));
            view.Vertical.SetPosition(1, new Vector3(0f, targetRadius * 1.5f, 0f));
        }

        // 자유 드로잉의 시작점과 끝점을 선으로 표시한다.
        private void ConfigureStroke(AnnotationView view, PilotAnnotation annotation)
        {
            view.Root.transform.localPosition = new Vector3(0f, 0f, overlayDepth);
            Vector2 start = NormalizedToLocal(annotation.start.x, annotation.start.y);
            Vector2 end = NormalizedToLocal(annotation.end.x, annotation.end.y);

            view.Primary.enabled = true;
            view.Primary.loop = false;
            view.Primary.positionCount = 2;
            view.Primary.SetPosition(0, new Vector3(start.x, start.y, 0f));
            view.Primary.SetPosition(1, new Vector3(end.x, end.y, 0f));
            view.Horizontal.enabled = false;
            view.Vertical.enabled = false;
        }

        // 비균일 FPV Quad 스케일에서도 목표 원을 원형으로 보정한다.
        private float HorizontalScaleCompensation()
        {
            if (overlayRoot == null || Mathf.Abs(overlayRoot.lossyScale.x) < 0.0001f)
            {
                return 1f;
            }

            return Mathf.Abs(overlayRoot.lossyScale.y / overlayRoot.lossyScale.x);
        }

        // 주석 표시용 LineRenderer 묶음을 가져오거나 생성한다.
        private AnnotationView GetOrCreateView(string id)
        {
            if (views.TryGetValue(id, out AnnotationView existing))
            {
                return existing;
            }

            AnnotationView view = viewPool.Count > 0 ? viewPool.Pop() : CreateView();
            view.Root.transform.SetParent(overlayRoot, false);
            views[id] = view;
            return view;
        }

        // 주석 표시용 세 개의 선 렌더러를 생성한다.
        private AnnotationView CreateView()
        {
            GameObject root = new GameObject("PilotAnnotation");
            root.transform.SetParent(overlayRoot, false);
            return new AnnotationView
            {
                Root = root,
                Primary = CreateLineRenderer(root.transform, "Primary"),
                Horizontal = CreateLineRenderer(root.transform, "Horizontal"),
                Vertical = CreateLineRenderer(root.transform, "Vertical")
            };
        }

        // 공통 설정을 적용한 선 렌더러를 생성한다.
        private LineRenderer CreateLineRenderer(Transform parent, string lineName)
        {
            GameObject lineObject = new GameObject(lineName);
            lineObject.transform.SetParent(parent, false);
            LineRenderer line = lineObject.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.alignment = LineAlignment.View;
            line.numCapVertices = 2;
            line.numCornerVertices = 2;
            line.startWidth = strokeWidth;
            line.endWidth = strokeWidth;
            line.startColor = annotationColor;
            line.endColor = annotationColor;
            line.sharedMaterial = GetLineMaterial();
            line.shadowCastingMode = ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.sortingOrder = 20;
            line.enabled = false;
            return line;
        }

        // 모든 주석 렌더러가 공유할 Unlit Material을 반환한다.
        private Material GetLineMaterial()
        {
            if (lineMaterial != null)
            {
                return lineMaterial;
            }

            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
            {
                shader = Shader.Find("Sprites/Default");
            }

            lineMaterial = new Material(shader)
            {
                name = "Pilot Annotation Runtime Material",
                color = annotationColor
            };
            if (lineMaterial.HasProperty("_BaseColor"))
            {
                lineMaterial.SetColor("_BaseColor", annotationColor);
            }

            return lineMaterial;
        }

        // 별도 공용 부모가 있으면 주석·타겟·YOLO 표시를 한 번에 전환한다.
        private void ApplyVisibility()
        {
            bool visible = IsEffectivelyVisible;
            if (overlayRoot != null && overlayRoot != transform)
            {
                overlayRoot.gameObject.SetActive(visible);
            }

            foreach (AnnotationView view in views.Values)
            {
                view.Root.SetActive(visible);
            }
        }

        // 같은 ID의 표시 순서를 최신으로 갱신한다.
        private void MoveToNewest(string id)
        {
            if (!orderNodes.TryGetValue(id, out LinkedListNode<string> node))
            {
                return;
            }

            annotationOrder.Remove(node);
            annotationOrder.AddLast(node);
        }

        // 제한 개수를 넘은 가장 오래된 주석을 제거한다.
        private void RemoveOldest()
        {
            LinkedListNode<string> oldest = annotationOrder.First;
            if (oldest != null)
            {
                RemoveAnnotation(oldest.Value);
            }
        }

        // 지정한 주석과 표시 오브젝트를 현재 상태에서 제거한다.
        private void RemoveAnnotation(string id)
        {
            annotations.Remove(id);
            if (views.Remove(id, out AnnotationView view))
            {
                ReturnView(view);
            }

            if (orderNodes.Remove(id, out LinkedListNode<string> node))
            {
                annotationOrder.Remove(node);
            }
        }

        // 사용하지 않는 표시 오브젝트를 풀에 반환한다.
        private void ReturnView(AnnotationView view)
        {
            view.Root.SetActive(false);
            viewPool.Push(view);
        }

        // 지원하는 주석 종류와 좌표 범위를 확인한다.
        private static bool IsValid(PilotAnnotation annotation)
        {
            if (annotation == null || string.IsNullOrWhiteSpace(annotation.id))
            {
                return false;
            }

            if (annotation.type == TargetType)
            {
                return PointValid(annotation.x, annotation.y);
            }

            return annotation.type == StrokeType
                && annotation.start != null
                && annotation.end != null
                && PointValid(annotation.start.x, annotation.start.y)
                && PointValid(annotation.end.x, annotation.end.y);
        }

        // 정규화 좌표가 영상 범위 안인지 확인한다.
        private static bool PointValid(float x, float y)
        {
            return x >= 0f && x <= 1f && y >= 0f && y <= 1f;
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
