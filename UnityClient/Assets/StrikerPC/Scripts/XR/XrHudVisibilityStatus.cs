using StrikerPC.Video;
using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    public sealed class XrHudVisibilityStatus : MonoBehaviour
    {
        public const float DefaultWidth = 112f;
        public const float DefaultHeight = 28f;
        public const float DefaultCornerPadding = 16f;
        public const float DefaultDepthOffset = -16f;

        [SerializeField] private PilotAnnotationOverlay pilotAnnotations;
        [SerializeField] private RectTransform statusRect;
        [SerializeField] private RectTransform surface;
        [SerializeField] private Graphic background;
        [SerializeField] private Text label;

        private bool subscribed;

        public bool IsShowingAdditionalInfo =>
            pilotAnnotations != null && pilotAnnotations.IsEffectivelyVisible;

        // FPV Canvas 우상단에 비상호작용 추가정보 상태 표시를 한 번만 생성한다.
        public static XrHudVisibilityStatus EnsureCreated(XrHudLayout targetLayout)
        {
            if (targetLayout == null || !Application.isPlaying
                || !targetLayout.TryGetWindow(XrWindowRole.Fpv, out XrDockableWindow fpv)
                || fpv == null || fpv.MovableWindow == null)
            {
                return null;
            }

            Canvas canvas = fpv.MovableWindow.GetComponentInChildren<Canvas>(true);
            RectTransform canvasRect = canvas != null ? canvas.transform as RectTransform : null;
            PilotAnnotationOverlay annotations =
                fpv.MovableWindow.GetComponentInChildren<PilotAnnotationOverlay>(true);
            if (canvasRect == null || annotations == null)
            {
                return null;
            }

            XrHudVisibilityStatus existing =
                canvasRect.GetComponentInChildren<XrHudVisibilityStatus>(true);
            if (existing != null)
            {
                existing.Configure(
                    annotations,
                    canvasRect,
                    existing.GetComponent<Graphic>(),
                    existing.GetComponentInChildren<Text>(true));
                return existing;
            }

            GameObject statusObject = new GameObject(
                "PilotAdditionalInfoStatus",
                typeof(RectTransform));
            statusObject.SetActive(false);
            statusObject.layer = canvas.gameObject.layer;
            RectTransform rect = (RectTransform)statusObject.transform;
            rect.SetParent(canvasRect, false);

            statusObject.AddComponent<CanvasRenderer>();
            Image image = statusObject.AddComponent<Image>();
            image.raycastTarget = false;

            GameObject labelObject = new GameObject("Label", typeof(RectTransform));
            labelObject.layer = statusObject.layer;
            RectTransform labelRect = (RectTransform)labelObject.transform;
            labelRect.SetParent(rect, false);
            labelRect.anchorMin = Vector2.zero;
            labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = Vector2.zero;
            labelRect.offsetMax = Vector2.zero;
            labelObject.AddComponent<CanvasRenderer>();
            Text text = labelObject.AddComponent<Text>();
            text.alignment = TextAnchor.MiddleCenter;
            text.fontSize = 13;
            text.fontStyle = FontStyle.Bold;
            text.raycastTarget = false;
            Text existingText = canvasRect.GetComponentInChildren<Text>(true);
            text.font = existingText != null && existingText.font != null
                ? existingText.font
                : Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

            XrHudVisibilityStatus status =
                statusObject.AddComponent<XrHudVisibilityStatus>();
            status.Configure(annotations, canvasRect, image, text);
            rect.SetAsLastSibling();
            statusObject.SetActive(true);
            return status;
        }

        // 상태 공급자와 FPV Canvas 표시 참조를 연결한다.
        public void Configure(
            PilotAnnotationOverlay annotations,
            RectTransform targetSurface,
            Graphic targetBackground,
            Text targetLabel)
        {
            Unsubscribe();
            pilotAnnotations = annotations;
            surface = targetSurface;
            background = targetBackground;
            label = targetLabel;
            ResolveReferences();
            ApplyGeometry();
            Subscribe();
            Refresh();
        }

        private void Awake()
        {
            ResolveReferences();
            ApplyGeometry();
        }

        private void OnEnable()
        {
            Subscribe();
            Refresh();
        }

        // 현재 파일럿 추가정보 표시 상태를 텍스트와 색상에 반영한다.
        public void Refresh()
        {
            bool visible = IsShowingAdditionalInfo;
            if (label != null)
            {
                label.text = FormatStatus(visible);
                label.color = Color.white;
            }
            if (background != null)
            {
                background.color = visible
                    ? new Color(0.1f, 0.8f, 0.9f, 0.16f)
                    : new Color(0.35f, 0.4f, 0.45f, 0.16f);
                background.raycastTarget = false;
            }
        }

        public static string FormatStatus(bool visible)
        {
            return visible ? "추가정보 ON" : "추가정보 OFF";
        }

        private void OnVisibilityChanged(bool _)
        {
            Refresh();
        }

        private void ResolveReferences()
        {
            statusRect ??= transform as RectTransform;
            surface ??= statusRect != null ? statusRect.parent as RectTransform : null;
            background ??= GetComponent<Graphic>();
            label ??= GetComponentInChildren<Text>(true);
            pilotAnnotations ??= GetComponentInParent<PilotAnnotationOverlay>();
        }

        // 상태 표시는 고정 크기로 FPV Canvas 우상단에 배치한다.
        private void ApplyGeometry()
        {
            if (statusRect == null || surface == null)
            {
                return;
            }

            statusRect.anchorMin = new Vector2(0.5f, 0.5f);
            statusRect.anchorMax = new Vector2(0.5f, 0.5f);
            statusRect.pivot = new Vector2(0.5f, 0.5f);
            statusRect.sizeDelta = new Vector2(DefaultWidth, DefaultHeight);
            Rect rect = surface.rect;
            statusRect.anchoredPosition3D = new Vector3(
                rect.xMax - DefaultCornerPadding - DefaultWidth * 0.5f,
                rect.yMax - DefaultCornerPadding - DefaultHeight * 0.5f,
                DefaultDepthOffset);
        }

        private void Subscribe()
        {
            if (subscribed || pilotAnnotations == null)
            {
                return;
            }

            pilotAnnotations.EffectiveVisibilityChanged += OnVisibilityChanged;
            subscribed = true;
        }

        private void Unsubscribe()
        {
            if (!subscribed || pilotAnnotations == null)
            {
                return;
            }

            pilotAnnotations.EffectiveVisibilityChanged -= OnVisibilityChanged;
            subscribed = false;
        }

        private void OnDisable()
        {
            Unsubscribe();
        }

        private void OnDestroy()
        {
            Unsubscribe();
        }
    }
}
