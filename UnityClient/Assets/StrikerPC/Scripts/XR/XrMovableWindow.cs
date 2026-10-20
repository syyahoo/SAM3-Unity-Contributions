using System;
using System.Collections;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Events;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Transformers;

namespace StrikerPC.XR
{
    [RequireComponent(typeof(BoxCollider))]
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(XRGrabInteractable))]
    [RequireComponent(typeof(XRGeneralGrabTransformer))]
    public sealed class XrMovableWindow : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
    {
        [Serializable]
        private struct SavedTransform
        {
            public Vector3 localPosition;
            public Quaternion localRotation;
            public Vector3 localScale;
            public Vector2 size;
            public bool pinned;
            public bool minimized;
        }

        [SerializeField] private string persistenceKey;
        [SerializeField] private bool pinned;
        [SerializeField] private float minimumScaleRatio = 0.4f;
        [SerializeField] private float maximumScaleRatio = 2.5f;
        [SerializeField] private UnityEvent<bool> pinnedChanged;
        [Header("Spatial Window Chrome")]
        [SerializeField] private RectTransform windowRect;
        [SerializeField] private RectTransform controlBar;
        [SerializeField] private BoxCollider controlBarCollider;
        [SerializeField] private Graphic controlBarBackground;
        [SerializeField] private CanvasGroup controlButtons;
        [SerializeField] private GameObject contentRoot;
        [SerializeField] private RectTransform[] synchronizedContentRects;
        [SerializeField] private Transform scaledContent;
        [SerializeField] private Vector2 scaledContentReferenceSize;
        [SerializeField] private Vector3 scaledContentReferenceScale = Vector3.one;
        [SerializeField] private bool rotateWhileDragging = true;
        [SerializeField] private bool allowTwoHandedManipulation = true;
        [SerializeField] private Vector2 minimumWindowSize = new Vector2(420f, 120f);
        [SerializeField] private Vector2 maximumWindowSize = new Vector2(1400f, 900f);
        [SerializeField, Min(1f)] private float collapsedBarWidth = 96f;
        [SerializeField, Min(1f)] private float expandedBarWidth = 280f;
        [SerializeField, Min(0f)] private float barAnimationSeconds = 0.16f;

        private XRGrabInteractable grabInteractable;
        private XRGeneralGrabTransformer grabTransformer;
        private BoxCollider rootCollider;
        private XrWindowResizeHandle[] resizeHandles;
        private Coroutine barAnimation;
        private bool minimized;
        private bool barHovered;
        private bool barSelected;
        private bool resizing;
        private bool focusPresentation;
        private bool controlBarWasActive;
        private Vector2 activeResizeCorner;
        private Vector3 resizeOppositeParentPosition;
        private Vector3 resizeRightAxisInParent;
        private Vector3 resizeUpAxisInParent;
        private Vector2 resizeStartSize;
        private float resizeAspectRatio = 1f;
        private bool scaledContentOffsetCaptured;
        private Vector3 scaledContentOffsetInWindowParent;
        private bool centerLocked;
        private bool centerLockedRectPositionCaptured;
        private Vector3 centerLockedRectLocalPosition;

        public bool IsPinned => pinned;
        public bool IsMinimized => minimized;
        public bool IsFocusPresentation => focusPresentation;

        // 중앙 FPV 리사이즈가 창의 시각 중심을 이동시키지 않도록 고정 상태를 적용한다.
        public void SetCenterLocked(bool enabled)
        {
            if (centerLocked == enabled)
            {
                if (enabled)
                {
                    RestoreCenterLockedRectPosition();
                }
                return;
            }

            centerLocked = enabled;
            centerLockedRectPositionCaptured = false;
            if (enabled)
            {
                CaptureCenterLockedRectPosition();
                RestoreCenterLockedRectPosition();
            }
        }

        // 저장된 배치는 바꾸지 않고 콘텐츠만 남기는 포커스 표시를 적용한다.
        public void SetFocusPresentation(bool enabled)
        {
            if (focusPresentation == enabled)
            {
                return;
            }

            focusPresentation = enabled;
            resizing = false;
            if (enabled)
            {
                controlBarWasActive = controlBar != null && controlBar.gameObject.activeSelf;
                if (barAnimation != null)
                {
                    StopCoroutine(barAnimation);
                    barAnimation = null;
                }
                if (controlBar != null)
                {
                    controlBar.gameObject.SetActive(false);
                }
            }
            else if (controlBar != null)
            {
                controlBar.gameObject.SetActive(controlBarWasActive);
            }

            ApplyPinnedState(false);
            ApplyMinimizedState();
            SetControlBarExpanded(!enabled && (pinned || barHovered || barSelected), true);
        }

        // 창 조작 구성과 저장된 배치를 복원한다.
        private void Awake()
        {
            grabInteractable = GetComponent<XRGrabInteractable>();
            grabTransformer = GetComponent<XRGeneralGrabTransformer>();
            rootCollider = GetComponent<BoxCollider>();
            resizeHandles = GetComponentsInChildren<XrWindowResizeHandle>(true);
            CaptureScaledContentOffset();
            ConfigureInteraction();
            SubscribeInteractionEvents();
            RestoreTransform();
            ApplyPinnedState(false);
            ApplyMinimizedState();
            SetControlBarExpanded(pinned, true);
        }

        // 창의 이동과 양손 크기 조절 범위를 구성한다.
        private void ConfigureInteraction()
        {
            Rigidbody body = GetComponent<Rigidbody>();
            body.isKinematic = true;
            body.useGravity = false;

            grabInteractable.selectMode = allowTwoHandedManipulation
                ? InteractableSelectMode.Multiple
                : InteractableSelectMode.Single;
            grabInteractable.throwOnDetach = false;
            grabInteractable.trackPosition = true;
            grabInteractable.trackRotation = rotateWhileDragging;
            grabInteractable.smoothPosition = true;
            grabInteractable.smoothRotation = true;

            bool usesSpatialChrome = windowRect != null && controlBar != null;
            BoxCollider dragCollider = controlBarCollider != null
                ? controlBarCollider
                : rootCollider;
            if (usesSpatialChrome && dragCollider != null)
            {
                grabInteractable.colliders.Clear();
                grabInteractable.colliders.Add(dragCollider);
                if (rootCollider != null && rootCollider != dragCollider)
                {
                    rootCollider.enabled = false;
                }
            }

            grabTransformer.allowOneHandedScaling = !usesSpatialChrome;
            grabTransformer.allowTwoHandedScaling = !usesSpatialChrome && allowTwoHandedManipulation;
            grabTransformer.clampScaling = true;
            grabTransformer.minimumScaleRatio = Mathf.Max(0.1f, minimumScaleRatio);
            grabTransformer.maximumScaleRatio = Mathf.Max(
                grabTransformer.minimumScaleRatio,
                maximumScaleRatio);

            UpdateGrabCollider();
        }

        // 창 이동용 Grab 이벤트를 연결한다.
        private void SubscribeInteractionEvents()
        {
            grabInteractable.hoverEntered.AddListener(OnGrabHoverEntered);
            grabInteractable.hoverExited.AddListener(OnGrabHoverExited);
            grabInteractable.selectEntered.AddListener(OnGrabSelected);
            grabInteractable.selectExited.AddListener(OnGrabReleased);
        }

        // 창의 고정 상태를 전환한다.
        public void TogglePinned()
        {
            SetPinned(!pinned);
        }

        // 창의 고정 상태를 지정한다.
        public void SetPinned(bool value)
        {
            if (pinned == value)
            {
                return;
            }

            pinned = value;
            ApplyPinnedState(true);
            SaveTransform();
        }

        // 창의 최소화 상태를 전환한다.
        public void ToggleMinimized()
        {
            SetMinimized(!minimized);
        }

        // 창 콘텐츠 표시 상태를 지정한다.
        public void SetMinimized(bool value)
        {
            if (minimized == value)
            {
                return;
            }

            minimized = value;
            ApplyMinimizedState();
            SaveTransform();
        }

        // 닫기 버튼으로 창 루트를 비활성화한다.
        public void CloseWindow()
        {
            SaveTransform();
            gameObject.SetActive(false);
        }

        // 고정 상태에 따라 잡기 기능을 변경한다.
        private void ApplyPinnedState(bool notify)
        {
            if (grabInteractable != null)
            {
                grabInteractable.enabled = !pinned && !focusPresentation;
            }

            ApplyResizeHandleState();
            SetControlBarExpanded(pinned || barHovered || barSelected, false);

            if (notify)
            {
                pinnedChanged?.Invoke(pinned);
            }
        }

        // 최소화 상태에 따라 콘텐츠와 리사이즈 핸들을 전환한다.
        private void ApplyMinimizedState()
        {
            if (contentRoot != null)
            {
                contentRoot.SetActive(focusPresentation || !minimized);
            }

            ApplyResizeHandleState();
        }

        // 고정 또는 최소화 중에는 모서리 리사이즈를 차단한다.
        private void ApplyResizeHandleState()
        {
            if (resizeHandles == null)
            {
                return;
            }

            bool enabled = !focusPresentation && !pinned && !minimized;
            foreach (XrWindowResizeHandle handle in resizeHandles)
            {
                if (handle != null)
                {
                    handle.SetInteractionEnabled(enabled);
                }
            }
        }

        // 모서리 드래그 시작 시 반대쪽 모서리를 고정점으로 저장한다.
        public bool BeginResize(Vector2 corner)
        {
            if (focusPresentation || pinned || minimized
                || windowRect == null || windowRect.parent == null)
            {
                return false;
            }

            Vector2 resizeDirection = NormalizeResizeDirection(corner);
            if (resizeDirection.x == 0f || resizeDirection.y == 0f)
            {
                return false;
            }

            CaptureScaledContentOffset();
            activeResizeCorner = resizeDirection;
            resizeStartSize = windowRect.rect.size;
            resizeAspectRatio = resizeStartSize.x / Mathf.Max(1f, resizeStartSize.y);
            Vector3 oppositeLocal = new Vector3(
                activeResizeCorner.x == 0f
                    ? 0f
                    : -activeResizeCorner.x * resizeStartSize.x * 0.5f,
                activeResizeCorner.y == 0f
                    ? 0f
                    : -activeResizeCorner.y * resizeStartSize.y * 0.5f,
                0f);
            Transform parent = windowRect.parent;
            resizeOppositeParentPosition = parent.InverseTransformPoint(
                windowRect.TransformPoint(oppositeLocal));
            resizeRightAxisInParent = parent.InverseTransformDirection(windowRect.right).normalized;
            resizeUpAxisInParent = parent.InverseTransformDirection(windowRect.up).normalized;
            resizing = true;
            return true;
        }

        // 선택한 모서리 위치에 맞춰 창 크기와 중심을 조절한다.
        public void ResizeFromHandle(Vector2 corner, Vector3 worldPosition)
        {
            if (!resizing || windowRect == null || windowRect.parent == null ||
                NormalizeResizeDirection(corner) != activeResizeCorner)
            {
                return;
            }

            Transform parent = windowRect.parent;
            Vector3 currentParentPosition = parent.InverseTransformPoint(worldPosition);
            Vector3 delta = currentParentPosition - resizeOppositeParentPosition;
            float scaleX = Mathf.Max(0.0001f, Mathf.Abs(windowRect.localScale.x));
            float scaleY = Mathf.Max(0.0001f, Mathf.Abs(windowRect.localScale.y));
            float candidateWidth = activeResizeCorner.x
                * Vector3.Dot(delta, resizeRightAxisInParent) / scaleX;
            float candidateHeight = activeResizeCorner.y
                * Vector3.Dot(delta, resizeUpAxisInParent) / scaleY;
            float widthRatio = candidateWidth / Mathf.Max(1f, resizeStartSize.x);
            float heightRatio = candidateHeight / Mathf.Max(1f, resizeStartSize.y);
            float sizeRatio = Mathf.Abs(widthRatio - 1f) >= Mathf.Abs(heightRatio - 1f)
                ? widthRatio
                : heightRatio;
            float minimumRatio = Mathf.Max(
                minimumWindowSize.x / Mathf.Max(1f, resizeStartSize.x),
                minimumWindowSize.y / Mathf.Max(1f, resizeStartSize.y));
            float maximumRatio = Mathf.Min(
                maximumWindowSize.x / Mathf.Max(1f, resizeStartSize.x),
                maximumWindowSize.y / Mathf.Max(1f, resizeStartSize.y));
            sizeRatio = Mathf.Clamp(sizeRatio, minimumRatio, maximumRatio);
            float widthInParent = resizeStartSize.x * sizeRatio * scaleX;
            float heightInParent = widthInParent / resizeAspectRatio * scaleY / scaleX;

            Vector3 center = resizeOppositeParentPosition
                + resizeRightAxisInParent
                    * (activeResizeCorner.x * widthInParent * 0.5f)
                + resizeUpAxisInParent
                    * (activeResizeCorner.y * heightInParent * 0.5f);
            center.z = windowRect.localPosition.z;
            if (centerLocked)
            {
                CaptureCenterLockedRectPosition();
                RestoreCenterLockedRectPosition();
            }
            else
            {
                windowRect.localPosition = center;
            }
            Vector2 size = new Vector2(
                widthInParent / scaleX,
                heightInParent / scaleY);
            windowRect.sizeDelta = size;
            SynchronizeContentSize(size);
            UpdateGrabCollider();
        }

        // 0인 축은 고정하고 나머지 축만 -1 또는 1 방향으로 정규화한다.
        private static Vector2 NormalizeResizeDirection(Vector2 direction)
        {
            return new Vector2(
                Mathf.Abs(direction.x) <= 0.0001f ? 0f : Mathf.Sign(direction.x),
                Mathf.Abs(direction.y) <= 0.0001f ? 0f : Mathf.Sign(direction.y));
        }

        // 모서리 드래그 종료 시 변경된 배치를 저장한다.
        public void EndResize()
        {
            if (!resizing)
            {
                return;
            }

            resizing = false;
            RestoreCenterLockedRectPosition();
            SaveTransform();
        }

        // 중앙 고정이 활성화된 시점의 UI 크롬 중심을 기준점으로 저장한다.
        private void CaptureCenterLockedRectPosition()
        {
            if (!centerLocked || centerLockedRectPositionCaptured || windowRect == null)
            {
                return;
            }

            centerLockedRectLocalPosition = windowRect.localPosition;
            centerLockedRectPositionCaptured = true;
        }

        // 크기 조절 중에도 중앙 FPV의 UI 크롬 중심을 기준점에 유지한다.
        private void RestoreCenterLockedRectPosition()
        {
            CaptureCenterLockedRectPosition();
            if (centerLocked && centerLockedRectPositionCaptured && windowRect != null)
            {
                windowRect.localPosition = centerLockedRectLocalPosition;
            }
        }

        // 현재 창 배치를 PlayerPrefs에 저장한다.
        public void SaveTransform()
        {
            SavedTransform state = new SavedTransform
            {
                localPosition = transform.localPosition,
                localRotation = transform.localRotation,
                localScale = transform.localScale,
                size = windowRect != null ? windowRect.sizeDelta : Vector2.zero,
                pinned = pinned,
                minimized = minimized
            };
            PlayerPrefs.SetString(GetPersistenceKey(), JsonUtility.ToJson(state));
            PlayerPrefs.Save();
        }

        // 고정 HUD는 Scene 기준 자세를 유지하되 저장된 눈높이 Y만 복원한다.
        public void RestoreTransform()
        {
            string key = GetPersistenceKey();
            string storedValue;
            if (!PlayerPrefs.HasKey(key))
            {
                string legacyKey = GetLegacyPersistenceKey();
                if (!PlayerPrefs.HasKey(legacyKey))
                {
                    return;
                }

                storedValue = PlayerPrefs.GetString(legacyKey);
                PlayerPrefs.SetString(key, storedValue);
                PlayerPrefs.Save();
            }
            else
            {
                storedValue = PlayerPrefs.GetString(key);
            }

            SavedTransform state = JsonUtility.FromJson<SavedTransform>(storedValue);
            XrDockableWindow dockableWindow = GetComponent<XrDockableWindow>();
            bool verticalHud = dockableWindow != null
                && dockableWindow.Role == XrWindowRole.Fpv;
            verticalHud |= GetComponent<XrVerticalHudAdjuster>() != null;
            if (verticalHud)
            {
                Vector3 restoredPosition = transform.localPosition;
                restoredPosition.y = state.localPosition.y;
                transform.localPosition = restoredPosition;
            }
            else if (dockableWindow == null)
            {
                transform.SetLocalPositionAndRotation(state.localPosition, state.localRotation);
                transform.localScale = state.localScale;
            }
            if (windowRect != null && state.size.x > 0f && state.size.y > 0f)
            {
                windowRect.sizeDelta = state.size;
                SynchronizeContentSize(state.size);
            }
            pinned = state.pinned;
            minimized = state.minimized;
            UpdateGrabCollider();
        }

        // XR 포인터가 창에 진입하면 하단 컨트롤 바를 확장한다.
        public void OnPointerEnter(PointerEventData eventData)
        {
            SetBarHovered(true);
        }

        // XR 포인터가 창에서 벗어나면 하단 컨트롤 바를 축소한다.
        public void OnPointerExit(PointerEventData eventData)
        {
            SetBarHovered(false);
        }

        // Grab 포인터 진입 시 하단 컨트롤 바를 확장한다.
        private void OnGrabHoverEntered(HoverEnterEventArgs args)
        {
            SetBarHovered(true);
        }

        // Grab 포인터 이탈 시 하단 컨트롤 바를 축소한다.
        private void OnGrabHoverExited(HoverExitEventArgs args)
        {
            SetBarHovered(false);
        }

        // 하단 바를 잡는 동안 컨트롤을 열린 상태로 유지한다.
        private void OnGrabSelected(SelectEnterEventArgs args)
        {
            barSelected = true;
            SetControlBarExpanded(true, false);
        }

        // 하단 바 드래그 종료 시 위치를 저장한다.
        private void OnGrabReleased(SelectExitEventArgs args)
        {
            barSelected = false;
            SetControlBarExpanded(pinned || barHovered, false);
            SaveTransform();
        }

        // 하단 바의 현재 hover 상태를 적용한다.
        private void SetBarHovered(bool value)
        {
            barHovered = value;
            SetControlBarExpanded(pinned || barHovered || barSelected, false);
        }

        // 하단 컨트롤 바의 폭과 버튼 표시를 전환한다.
        private void SetControlBarExpanded(bool expanded, bool immediate)
        {
            if (controlBar == null || controlButtons == null)
            {
                return;
            }

            if (barAnimation != null)
            {
                StopCoroutine(barAnimation);
            }

            if (immediate || barAnimationSeconds <= 0f || !isActiveAndEnabled)
            {
                ApplyControlBarVisual(expanded ? 1f : 0f);
                return;
            }

            barAnimation = StartCoroutine(AnimateControlBar(expanded));
        }

        // 하단 컨트롤 바를 부드럽게 확장하거나 축소한다.
        private IEnumerator AnimateControlBar(bool expanded)
        {
            float start = Mathf.InverseLerp(
                collapsedBarWidth,
                expandedBarWidth,
                controlBar.sizeDelta.x);
            float target = expanded ? 1f : 0f;
            float elapsed = 0f;
            while (elapsed < barAnimationSeconds)
            {
                elapsed += Time.unscaledDeltaTime;
                ApplyControlBarVisual(Mathf.Lerp(start, target, elapsed / barAnimationSeconds));
                yield return null;
            }

            ApplyControlBarVisual(target);
            barAnimation = null;
        }

        // 하단 컨트롤 바의 현재 시각 상태를 반영한다.
        private void ApplyControlBarVisual(float expansion)
        {
            Vector2 size = controlBar.sizeDelta;
            size.x = Mathf.Lerp(collapsedBarWidth, expandedBarWidth, expansion);
            controlBar.sizeDelta = size;
            controlButtons.alpha = expansion;
            controlButtons.interactable = expansion >= 0.95f;
            controlButtons.blocksRaycasts = expansion >= 0.5f;
            if (controlBarBackground != null)
            {
                Color color = controlBarBackground.color;
                color.a = 0.96f;
                controlBarBackground.color = color;
            }
            UpdateGrabCollider();
        }

        // 패널 종류에 맞춰 UI Rect 또는 영상 Mesh 크기를 동기화한다.
        private void SynchronizeContentSize(Vector2 size)
        {
            if (synchronizedContentRects != null)
            {
                foreach (RectTransform target in synchronizedContentRects)
                {
                    if (target != null && target != windowRect)
                    {
                        target.sizeDelta = size;
                    }
                }
            }

            if (scaledContent == null || scaledContentReferenceSize.x <= 0f ||
                scaledContentReferenceSize.y <= 0f)
            {
                return;
            }

            scaledContent.localScale = new Vector3(
                scaledContentReferenceScale.x * size.x / scaledContentReferenceSize.x,
                scaledContentReferenceScale.y * size.y / scaledContentReferenceSize.y,
                scaledContentReferenceScale.z);
            CaptureScaledContentOffset();
            if (scaledContentOffsetCaptured && windowRect.parent != null)
            {
                Vector3 alignedPosition = windowRect.localPosition
                    + scaledContentOffsetInWindowParent;
                scaledContent.position = windowRect.parent.TransformPoint(alignedPosition);
            }
        }

        // 영상 Mesh와 UI 크롬의 초기 중심 간격을 리사이즈 기준으로 저장한다.
        private void CaptureScaledContentOffset()
        {
            if (scaledContentOffsetCaptured || scaledContent == null ||
                windowRect == null || windowRect.parent == null)
            {
                return;
            }

            Vector3 contentPosition = windowRect.parent.InverseTransformPoint(
                scaledContent.position);
            scaledContentOffsetInWindowParent = contentPosition - windowRect.localPosition;
            scaledContentOffsetCaptured = true;
        }

        // 하단 바 크기에 맞춰 이동용 Collider를 갱신한다.
        private void UpdateGrabCollider()
        {
            BoxCollider dragCollider = controlBarCollider != null
                ? controlBarCollider
                : rootCollider;
            if (dragCollider == null || controlBar == null)
            {
                return;
            }

            dragCollider.center = controlBarCollider != null
                ? Vector3.zero
                : controlBar.localPosition;
            dragCollider.size = new Vector3(
                Mathf.Max(1f, controlBar.rect.width),
                Mathf.Max(1f, controlBar.rect.height),
                24f);
        }

        // 창별로 안정적인 저장 키를 반환한다.
        private string GetPersistenceKey()
        {
            if (!string.IsNullOrWhiteSpace(persistenceKey))
            {
                return $"StrikerPC.XrWindow.{persistenceKey}";
            }

            string sceneName = gameObject.scene.name == "StrikerPC"
                ? "StrikerPC-Demo"
                : gameObject.scene.name;
            return $"StrikerPC.XrWindow.{sceneName}.{gameObject.name}";
        }

        // 기존 창 배치 키를 새 이름으로 한 번 마이그레이션할 때 사용한다.
        private string GetLegacyPersistenceKey()
        {
            if (!string.IsNullOrWhiteSpace(persistenceKey))
            {
                return $"DroneOps.XrWindow.{persistenceKey}";
            }

            string sceneName = gameObject.scene.name == "StrikerPC"
                || gameObject.scene.name == "StrikerPC-Demo"
                ? "DroneOps-Demo"
                : gameObject.scene.name;
            return $"DroneOps.XrWindow.{sceneName}.{gameObject.name}";
        }

        // 조작이 끝나거나 비활성화될 때 현재 배치를 저장한다.
        private void OnDisable()
        {
            if (Application.isPlaying)
            {
                SaveTransform();
            }
        }

        // 등록한 XR 이벤트를 해제한다.
        private void OnDestroy()
        {
            if (grabInteractable == null)
            {
                return;
            }

            grabInteractable.hoverEntered.RemoveListener(OnGrabHoverEntered);
            grabInteractable.hoverExited.RemoveListener(OnGrabHoverExited);
            grabInteractable.selectEntered.RemoveListener(OnGrabSelected);
            grabInteractable.selectExited.RemoveListener(OnGrabReleased);
        }
    }
}
