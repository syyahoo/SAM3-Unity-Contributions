using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

namespace StrikerPC.XR
{
    public enum XrHudToggleCorner
    {
        TopLeft = 0,
        TopRight = 1,
        BottomLeft = 2,
        BottomRight = 3
    }

    [DisallowMultipleComponent]
    [RequireComponent(typeof(Button))]
    [RequireComponent(typeof(BoxCollider))]
    [RequireComponent(typeof(XRSimpleInteractable))]
    public sealed class XrHudVisibilityToggle : MonoBehaviour
    {
        public const float DefaultButtonWidth = 64f;
        public const float DefaultButtonHeight = 28f;
        public const float DefaultCornerPadding = 16f;
        public const float DefaultDepthOffset = -16f;
        public const float DefaultHitDepth = 8f;

        private const float DragThreshold = 8f;
        private const string PersistencePrefix = "StrikerPC.XrHudVisibilityToggle";

        private static readonly XrWindowSlot[] PeripheralSlots =
        {
            XrWindowSlot.Left,
            XrWindowSlot.Right,
            XrWindowSlot.Upper,
            XrWindowSlot.Lower
        };

        [SerializeField] private XrHudLayout layout;
        [SerializeField] private RectTransform dragSurface;
        [SerializeField] private RectTransform buttonRect;
        [SerializeField] private Button button;
        [SerializeField] private Graphic background;
        [SerializeField] private Text label;
        [SerializeField] private BoxCollider hitCollider;
        [SerializeField] private XRSimpleInteractable interactable;
        [SerializeField] private XrHudToggleCorner currentCorner = XrHudToggleCorner.TopRight;

        private readonly bool[] peripheralVisibility = new bool[PeripheralSlots.Length];
        private IXRSelectInteractor selectingInteractor;
        private Vector2 pointerToButtonOffset;
        private Vector2 dragStartPosition;
        private Vector2 lastSurfaceSize = new Vector2(-1f, -1f);
        private bool visibilitySnapshotCaptured;
        private bool uiVisible = true;
        private bool dragging;
        private bool hovered;
        private bool eventsSubscribed;

        public bool IsUiVisible => uiVisible;
        public XrHudToggleCorner CurrentCorner => currentCorner;
        public static Vector2 DefaultButtonSize =>
            new Vector2(DefaultButtonWidth, DefaultButtonHeight);

        // FPV 월드 스페이스 Canvas에 토글 버튼을 한 번만 생성한다.
        public static XrHudVisibilityToggle EnsureCreated(XrHudLayout targetLayout)
        {
            if (targetLayout == null || !Application.isPlaying
                || !targetLayout.TryGetWindow(XrWindowRole.Fpv, out XrDockableWindow fpv)
                || fpv == null || fpv.MovableWindow == null)
            {
                return null;
            }

            Canvas canvas = fpv.MovableWindow.GetComponentInChildren<Canvas>(true);
            RectTransform canvasRect = canvas != null ? canvas.transform as RectTransform : null;
            if (canvasRect == null)
            {
                return null;
            }

            XrHudVisibilityToggle existing =
                canvasRect.GetComponentInChildren<XrHudVisibilityToggle>(true);
            if (existing != null)
            {
                existing.Configure(targetLayout, canvasRect, existing.GetComponent<Button>(),
                    existing.GetComponent<Graphic>(), existing.GetComponentInChildren<Text>(true));
                return existing;
            }

            GameObject toggleObject = new GameObject(
                "FpvUiVisibilityToggle",
                typeof(RectTransform));
            toggleObject.SetActive(false);
            toggleObject.layer = canvas.gameObject.layer;
            RectTransform toggleRect = (RectTransform)toggleObject.transform;
            toggleRect.SetParent(canvasRect, false);
            toggleRect.anchorMin = new Vector2(0.5f, 0.5f);
            toggleRect.anchorMax = new Vector2(0.5f, 0.5f);
            toggleRect.pivot = new Vector2(0.5f, 0.5f);
            toggleRect.sizeDelta = DefaultButtonSize;

            toggleObject.AddComponent<CanvasRenderer>();
            Image image = toggleObject.AddComponent<Image>();
            image.color = new Color(0.1f, 0.8f, 0.9f, 0.045f);
            image.raycastTarget = false;
            Button toggleButton = toggleObject.AddComponent<Button>();
            toggleButton.targetGraphic = image;
            toggleButton.transition = Selectable.Transition.None;
            toggleObject.AddComponent<BoxCollider>();
            toggleObject.AddComponent<XRSimpleInteractable>();

            GameObject labelObject = new GameObject("Label", typeof(RectTransform));
            labelObject.layer = toggleObject.layer;
            RectTransform labelRect = (RectTransform)labelObject.transform;
            labelRect.SetParent(toggleRect, false);
            labelRect.anchorMin = Vector2.zero;
            labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = Vector2.zero;
            labelRect.offsetMax = Vector2.zero;
            labelObject.AddComponent<CanvasRenderer>();
            Text text = labelObject.AddComponent<Text>();
            text.text = "UI";
            text.alignment = TextAnchor.MiddleCenter;
            text.fontSize = 14;
            text.fontStyle = FontStyle.Bold;
            text.raycastTarget = false;
            Text existingText = canvasRect.GetComponentInChildren<Text>(true);
            text.font = existingText != null && existingText.font != null
                ? existingText.font
                : Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

            XrHudVisibilityToggle toggle =
                toggleObject.AddComponent<XrHudVisibilityToggle>();
            toggle.Configure(targetLayout, canvasRect, toggleButton, image, text);
            toggleRect.SetAsLastSibling();
            toggleObject.SetActive(true);
            return toggle;
        }

        // 런타임 생성 시 필요한 참조와 고정 픽셀 크기를 연결한다.
        public void Configure(
            XrHudLayout targetLayout,
            RectTransform targetDragSurface,
            Button targetButton,
            Graphic targetBackground,
            Text targetLabel)
        {
            layout = targetLayout;
            dragSurface = targetDragSurface;
            button = targetButton;
            background = targetBackground;
            label = targetLabel;
            ResolveReferences();
            ConfigureInteractionGeometry();
            LoadCorner();
            ApplyCorner();
            ApplyVisual();
        }

        private void Awake()
        {
            ResolveReferences();
            ConfigureInteractionGeometry();
            LoadCorner();
            ApplyCorner();
            SubscribeEvents();
            ApplyVisual();
        }

        private void OnEnable()
        {
            ApplyCorner();
            ApplyVisual();
        }

        // XR 포인터 이동을 Canvas 좌표로 변환해 버튼을 드래그한다.
        private void Update()
        {
            SynchronizeSurfaceSize();
            if (selectingInteractor == null || dragSurface == null || buttonRect == null)
            {
                return;
            }

            Transform attachTransform = selectingInteractor.GetAttachTransform(interactable);
            if (attachTransform == null)
            {
                return;
            }

            Vector2 localPointer = dragSurface.InverseTransformPoint(attachTransform.position);
            Vector2 candidate = localPointer + pointerToButtonOffset;
            if (!dragging
                && (candidate - dragStartPosition).sqrMagnitude >= DragThreshold * DragThreshold)
            {
                dragging = true;
            }

            if (dragging)
            {
                SetButtonPosition(ClampToSurface(candidate));
            }
        }

        // FPV 창 크기가 변하면 고정 픽셀 버튼을 선택한 코너로 다시 정렬한다.
        private void SynchronizeSurfaceSize()
        {
            if (dragSurface == null)
            {
                return;
            }

            Vector2 surfaceSize = dragSurface.rect.size;
            if (surfaceSize == lastSurfaceSize)
            {
                return;
            }

            lastSurfaceSize = surfaceSize;
            if (selectingInteractor == null)
            {
                ApplyCorner();
            }
        }

        // FPV를 유지하고 주변 네 슬롯 UI만 켜거나 끈다.
        public void SetUiVisible(bool visible)
        {
            if (layout == null || uiVisible == visible)
            {
                return;
            }

            if (!visible)
            {
                for (int i = 0; i < PeripheralSlots.Length; i++)
                {
                    Transform anchor = layout.GetAnchor(PeripheralSlots[i]);
                    peripheralVisibility[i] = anchor != null && anchor.gameObject.activeSelf;
                    if (anchor != null)
                    {
                        anchor.gameObject.SetActive(false);
                    }
                }
                visibilitySnapshotCaptured = true;
            }
            else if (visibilitySnapshotCaptured)
            {
                RestorePeripheralVisibility();
            }

            uiVisible = visible;
            ApplyVisual();
        }

        public void ToggleUiVisibility()
        {
            SetUiVisible(!uiVisible);
        }

        // 지정한 코너로 스냅하고 필요할 때 다음 실행용 위치를 저장한다.
        public void MoveToCorner(XrHudToggleCorner corner, bool persist = true)
        {
            currentCorner = corner;
            ApplyCorner();
            if (!persist)
            {
                return;
            }

            PlayerPrefs.SetInt(GetPersistenceKey(), (int)currentCorner);
            PlayerPrefs.Save();
        }

        // Canvas Rect와 고정 버튼 크기로 코너 중심 좌표를 계산한다.
        public static Vector3 CalculateCornerPosition(
            Rect surfaceRect,
            Vector2 buttonSize,
            XrHudToggleCorner corner,
            float padding = DefaultCornerPadding,
            float depth = DefaultDepthOffset)
        {
            float halfWidth = buttonSize.x * 0.5f;
            float halfHeight = buttonSize.y * 0.5f;
            bool left = corner == XrHudToggleCorner.TopLeft
                || corner == XrHudToggleCorner.BottomLeft;
            bool top = corner == XrHudToggleCorner.TopLeft
                || corner == XrHudToggleCorner.TopRight;
            return new Vector3(
                left
                    ? surfaceRect.xMin + padding + halfWidth
                    : surfaceRect.xMax - padding - halfWidth,
                top
                    ? surfaceRect.yMax - padding - halfHeight
                    : surfaceRect.yMin + padding + halfHeight,
                depth);
        }

        private void OnHoverEntered(HoverEnterEventArgs args)
        {
            hovered = true;
            ApplyVisual();
        }

        private void OnHoverExited(HoverExitEventArgs args)
        {
            hovered = false;
            ApplyVisual();
        }

        private void OnSelectEntered(SelectEnterEventArgs args)
        {
            selectingInteractor = args.interactorObject;
            dragging = false;
            if (dragSurface == null || buttonRect == null)
            {
                ApplyVisual();
                return;
            }

            Transform attachTransform = selectingInteractor.GetAttachTransform(interactable);
            Vector2 pointer = attachTransform != null
                ? (Vector2)dragSurface.InverseTransformPoint(attachTransform.position)
                : buttonRect.anchoredPosition;
            dragStartPosition = buttonRect.anchoredPosition;
            pointerToButtonOffset = dragStartPosition - pointer;
            ApplyVisual();
        }

        private void OnSelectExited(SelectExitEventArgs args)
        {
            if (selectingInteractor != args.interactorObject)
            {
                return;
            }

            selectingInteractor = null;
            if (dragging)
            {
                SnapToClosestCorner();
            }
            else if (button != null && button.IsInteractable())
            {
                button.onClick.Invoke();
            }

            dragging = false;
            ApplyVisual();
        }

        private void SnapToClosestCorner()
        {
            if (dragSurface == null || buttonRect == null)
            {
                return;
            }

            XrHudToggleCorner closest = XrHudToggleCorner.TopRight;
            float closestDistance = float.PositiveInfinity;
            foreach (XrHudToggleCorner corner in System.Enum.GetValues(typeof(XrHudToggleCorner)))
            {
                Vector3 position = CalculateCornerPosition(
                    dragSurface.rect,
                    buttonRect.rect.size,
                    corner);
                float distance = ((Vector2)position - buttonRect.anchoredPosition).sqrMagnitude;
                if (distance < closestDistance)
                {
                    closest = corner;
                    closestDistance = distance;
                }
            }

            MoveToCorner(closest);
        }

        private Vector2 ClampToSurface(Vector2 position)
        {
            Rect rect = dragSurface.rect;
            Vector2 size = buttonRect.rect.size;
            float halfWidth = size.x * 0.5f;
            float halfHeight = size.y * 0.5f;
            return new Vector2(
                Mathf.Clamp(
                    position.x,
                    rect.xMin + DefaultCornerPadding + halfWidth,
                    rect.xMax - DefaultCornerPadding - halfWidth),
                Mathf.Clamp(
                    position.y,
                    rect.yMin + DefaultCornerPadding + halfHeight,
                    rect.yMax - DefaultCornerPadding - halfHeight));
        }

        private void ApplyCorner()
        {
            if (dragSurface == null || buttonRect == null)
            {
                return;
            }

            buttonRect.sizeDelta = DefaultButtonSize;
            Vector3 position = CalculateCornerPosition(
                dragSurface.rect,
                DefaultButtonSize,
                currentCorner);
            buttonRect.anchoredPosition3D = position;
            lastSurfaceSize = dragSurface.rect.size;
        }

        private void SetButtonPosition(Vector2 position)
        {
            Vector3 current = buttonRect.anchoredPosition3D;
            current.x = position.x;
            current.y = position.y;
            current.z = DefaultDepthOffset;
            buttonRect.anchoredPosition3D = current;
        }

        private void ConfigureInteractionGeometry()
        {
            if (buttonRect == null || hitCollider == null || interactable == null)
            {
                return;
            }

            buttonRect.anchorMin = new Vector2(0.5f, 0.5f);
            buttonRect.anchorMax = new Vector2(0.5f, 0.5f);
            buttonRect.pivot = new Vector2(0.5f, 0.5f);
            buttonRect.sizeDelta = DefaultButtonSize;
            hitCollider.center = Vector3.zero;
            hitCollider.size = new Vector3(
                DefaultButtonWidth,
                DefaultButtonHeight,
                DefaultHitDepth);
            hitCollider.isTrigger = true;
            interactable.colliders.Clear();
            interactable.colliders.Add(hitCollider);
            if (background != null)
            {
                background.raycastTarget = false;
            }
        }

        private void ResolveReferences()
        {
            buttonRect ??= transform as RectTransform;
            dragSurface ??= buttonRect != null ? buttonRect.parent as RectTransform : null;
            layout ??= GetComponentInParent<XrHudLayout>();
            button ??= GetComponent<Button>();
            background ??= button != null ? button.targetGraphic : GetComponent<Graphic>();
            label ??= GetComponentInChildren<Text>(true);
            hitCollider ??= GetComponent<BoxCollider>();
            interactable ??= GetComponent<XRSimpleInteractable>();
        }

        private void SubscribeEvents()
        {
            if (eventsSubscribed || interactable == null || button == null)
            {
                return;
            }

            interactable.hoverEntered.AddListener(OnHoverEntered);
            interactable.hoverExited.AddListener(OnHoverExited);
            interactable.selectEntered.AddListener(OnSelectEntered);
            interactable.selectExited.AddListener(OnSelectExited);
            button.onClick.AddListener(ToggleUiVisibility);
            eventsSubscribed = true;
        }

        private void RestorePeripheralVisibility()
        {
            if (layout == null)
            {
                return;
            }

            for (int i = 0; i < PeripheralSlots.Length; i++)
            {
                Transform anchor = layout.GetAnchor(PeripheralSlots[i]);
                if (anchor != null)
                {
                    anchor.gameObject.SetActive(peripheralVisibility[i]);
                }
            }
        }

        private void ApplyVisual()
        {
            if (background != null)
            {
                float alpha = selectingInteractor != null ? 0.18f : hovered ? 0.12f : 0.045f;
                background.color = uiVisible
                    ? new Color(0.1f, 0.8f, 0.9f, alpha)
                    : new Color(0.6f, 0.65f, 0.7f, alpha);
            }
            if (label != null)
            {
                label.text = "UI";
                label.color = Color.white;
            }
        }

        private void LoadCorner()
        {
            string key = GetPersistenceKey();
            if (!PlayerPrefs.HasKey(key))
            {
                return;
            }

            int storedCorner = PlayerPrefs.GetInt(key);
            if (System.Enum.IsDefined(typeof(XrHudToggleCorner), storedCorner))
            {
                currentCorner = (XrHudToggleCorner)storedCorner;
            }
        }

        // 씬 이름 변경 전의 HUD 토글 위치 키를 유지한다.
        private string GetPersistenceKey()
        {
            string sceneName = layout != null
                ? layout.gameObject.scene.name
                : gameObject.scene.name;
            if (sceneName == "StrikerPC")
            {
                sceneName = "StrikerPC-Demo";
            }

            return $"{PersistencePrefix}.{sceneName}.Corner";
        }

        private void OnDisable()
        {
            selectingInteractor = null;
            dragging = false;
            hovered = false;
        }

        private void OnDestroy()
        {
            if (!uiVisible && visibilitySnapshotCaptured)
            {
                RestorePeripheralVisibility();
            }
            if (!eventsSubscribed || interactable == null || button == null)
            {
                return;
            }

            interactable.hoverEntered.RemoveListener(OnHoverEntered);
            interactable.hoverExited.RemoveListener(OnHoverExited);
            interactable.selectEntered.RemoveListener(OnSelectEntered);
            interactable.selectExited.RemoveListener(OnSelectExited);
            button.onClick.RemoveListener(ToggleUiVisibility);
        }
    }
}
