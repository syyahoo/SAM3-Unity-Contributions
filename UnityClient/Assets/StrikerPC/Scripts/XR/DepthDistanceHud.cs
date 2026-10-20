using StrikerPC.Video;
using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.XR
{
    public enum DepthDistanceDisplayLevel
    {
        Unavailable = 0,
        Clear = 1,
        Caution = 2,
        Danger = 3
    }

    [DisallowMultipleComponent]
    public sealed class DepthDistanceHud : MonoBehaviour
    {
        public const float DangerThresholdMeters = 0.8f;
        public const float CautionThresholdMeters = 1.5f;
        public const float MaximumSampleAgeSeconds = 0.5f;
        public const ulong MaximumFrameDeltaNs = 300_000_000UL;

        private static readonly Color ClearColor = new Color(0.25f, 0.93f, 1f, 1f);
        private static readonly Color CautionColor = new Color(1f, 0.60f, 0.16f, 1f);
        private static readonly Color DangerColor = new Color(1f, 0.24f, 0.20f, 1f);
        private static readonly Color UnavailableColor = new Color(0.72f, 0.78f, 0.82f, 1f);

        [SerializeField] private DepthDistanceUdpReceiver receiver;
        [SerializeField] private PilotSharedFrameTexture frameSource;
        [SerializeField] private Text leftText;
        [SerializeField] private Text centerText;
        [SerializeField] private Text rightText;
        [SerializeField] private Text captionText;
        [SerializeField] private CanvasGroup visibility;

        private float nextFrameSourceLookupAt;
        private bool showingUnavailable;

        // Center FPV 월드 Canvas 하단에 비상호작용 거리 HUD를 한 번만 생성한다.
        public static DepthDistanceHud EnsureCreated(XrHudLayout targetLayout)
        {
            if (targetLayout == null
                || !Application.isPlaying
                || !targetLayout.TryGetWindow(XrWindowRole.Fpv, out XrDockableWindow fpv)
                || fpv == null
                || fpv.MovableWindow == null)
            {
                return null;
            }

            Canvas canvas = fpv.MovableWindow.GetComponentInChildren<Canvas>(true);
            RectTransform canvasRect = canvas != null ? canvas.transform as RectTransform : null;
            if (canvasRect == null)
            {
                return null;
            }

            DepthDistanceUdpReceiver targetReceiver =
                targetLayout.GetComponent<DepthDistanceUdpReceiver>();
            if (targetReceiver == null)
            {
                targetReceiver = targetLayout.gameObject.AddComponent<DepthDistanceUdpReceiver>();
            }
            PilotSharedFrameTexture targetFrameSource =
                fpv.MovableWindow.GetComponentInChildren<PilotSharedFrameTexture>(true);
            targetFrameSource ??= FindFirstObjectByType<PilotSharedFrameTexture>();

            DepthDistanceHud existing =
                canvasRect.GetComponentInChildren<DepthDistanceHud>(true);
            if (existing != null)
            {
                existing.Configure(
                    targetReceiver,
                    targetFrameSource,
                    existing.leftText,
                    existing.centerText,
                    existing.rightText,
                    existing.captionText);
                return existing;
            }

            Font font = ResolveFont(canvasRect);
            GameObject panelObject = new GameObject(
                "FpvDepthDistanceHud",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Image),
                typeof(CanvasGroup));
            panelObject.SetActive(false);
            panelObject.layer = canvas.gameObject.layer;
            RectTransform panelRect = (RectTransform)panelObject.transform;
            panelRect.SetParent(canvasRect, false);
            panelRect.anchorMin = new Vector2(0.5f, 0f);
            panelRect.anchorMax = new Vector2(0.5f, 0f);
            panelRect.pivot = new Vector2(0.5f, 0f);
            panelRect.sizeDelta = new Vector2(520f, 62f);
            panelRect.anchoredPosition3D = new Vector3(0f, 14f, -14f);

            Image background = panelObject.GetComponent<Image>();
            background.color = new Color(0.015f, 0.035f, 0.055f, 0.82f);
            background.raycastTarget = false;

            Text left = CreateText(
                panelRect,
                "LeftDistance",
                new Vector2(0f, 0.34f),
                new Vector2(0.33f, 1f),
                19,
                FontStyle.Bold,
                font);
            Text center = CreateText(
                panelRect,
                "CenterDistance",
                new Vector2(0.33f, 0.34f),
                new Vector2(0.67f, 1f),
                22,
                FontStyle.Bold,
                font);
            Text right = CreateText(
                panelRect,
                "RightDistance",
                new Vector2(0.67f, 0.34f),
                new Vector2(1f, 1f),
                19,
                FontStyle.Bold,
                font);
            Text caption = CreateText(
                panelRect,
                "Caption",
                new Vector2(0f, 0f),
                new Vector2(1f, 0.34f),
                12,
                FontStyle.Normal,
                font);

            DepthDistanceHud hud = panelObject.AddComponent<DepthDistanceHud>();
            hud.Configure(targetReceiver, targetFrameSource, left, center, right, caption);
            panelRect.SetAsLastSibling();
            panelObject.SetActive(true);
            return hud;
        }

        // 런타임 생성 HUD의 수신기와 Text 참조를 연결한다.
        public void Configure(
            DepthDistanceUdpReceiver targetReceiver,
            PilotSharedFrameTexture targetFrameSource,
            Text targetLeftText,
            Text targetCenterText,
            Text targetRightText,
            Text targetCaptionText)
        {
            receiver = targetReceiver;
            frameSource = targetFrameSource;
            leftText = targetLeftText;
            centerText = targetCenterText;
            rightText = targetRightText;
            captionText = targetCaptionText;
            visibility = GetComponent<CanvasGroup>();
            if (visibility != null)
            {
                visibility.interactable = false;
                visibility.blocksRaycasts = false;
                visibility.alpha = receiver != null && receiver.HasSample ? 1f : 0f;
            }
            ApplyUnavailable();
        }

        // FPV와 깊이 표본이 모두 최신일 때만 현재 거리로 표시한다.
        private void Update()
        {
            ResolveFrameSource();
            bool videoFresh = frameSource != null
                && frameSource.HasFreshFrame(MaximumSampleAgeSeconds);
            if (!videoFresh
                || receiver == null
                || !receiver.TryGetFreshSample(
                    Time.unscaledTime,
                    MaximumSampleAgeSeconds,
                    frameSource.LastSourceTimestampNs,
                    MaximumFrameDeltaNs,
                    out DepthDistanceSample sample))
            {
                SetVisible(receiver != null && receiver.HasSample);
                ApplyUnavailable();
                return;
            }

            SetVisible(true);
            showingUnavailable = false;
            ApplyDistance(leftText, "L", sample.LeftMeters);
            ApplyDistance(centerText, "C", sample.CenterMeters);
            ApplyDistance(rightText, "R", sample.RightMeters);
            float nearest = Mathf.Min(
                sample.LeftMeters,
                Mathf.Min(sample.CenterMeters, sample.RightMeters));
            if (captionText != null)
            {
                captionText.text = $"추정 거리 · 보조용 · {GetStatusLabel(Classify(nearest))}";
                captionText.color = new Color(0.82f, 0.88f, 0.92f, 1f);
            }
        }

        // 고정 임계값으로 한 구간의 거리 표시 단계를 판정한다.
        public static DepthDistanceDisplayLevel Classify(float meters)
        {
            if (float.IsNaN(meters) || float.IsInfinity(meters) || meters <= 0f)
            {
                return DepthDistanceDisplayLevel.Unavailable;
            }
            if (meters < DangerThresholdMeters)
            {
                return DepthDistanceDisplayLevel.Danger;
            }
            if (meters <= CautionThresholdMeters)
            {
                return DepthDistanceDisplayLevel.Caution;
            }
            return DepthDistanceDisplayLevel.Clear;
        }

        // 거리 수치는 숨기고 방향 라벨만 HUD 문자열로 만든다.
        public static string FormatDistance(string prefix, float meters)
        {
            _ = meters;
            return prefix;
        }

        // 거리 한 구간의 값과 단계 색상을 함께 적용한다.
        private static void ApplyDistance(Text target, string prefix, float meters)
        {
            if (target == null)
            {
                return;
            }
            DepthDistanceDisplayLevel level = Classify(meters);
            target.text = FormatDistance(prefix, meters);
            target.color = GetColor(level);
        }

        // stale 또는 낮은 신뢰 상태에서 이전 값을 즉시 제거한다.
        private void ApplyUnavailable()
        {
            if (showingUnavailable)
            {
                return;
            }
            showingUnavailable = true;
            ApplyDistance(leftText, "L", float.NaN);
            ApplyDistance(centerText, "C", float.NaN);
            ApplyDistance(rightText, "R", float.NaN);
            if (captionText != null)
            {
                captionText.text = "추정 거리 · 보조용 · 신뢰 부족";
                captionText.color = UnavailableColor;
            }
        }

        // 모듈을 한 번도 실행하지 않은 기본 FPV에는 거리 패널을 노출하지 않는다.
        private void SetVisible(bool visible)
        {
            if (visibility != null)
            {
                visibility.alpha = visible ? 1f : 0f;
            }
        }

        // FPV Texture가 늦게 생성된 경우 1초 간격으로만 참조를 다시 찾는다.
        private void ResolveFrameSource()
        {
            if (frameSource != null || Time.unscaledTime < nextFrameSourceLookupAt)
            {
                return;
            }
            nextFrameSourceLookupAt = Time.unscaledTime + 1f;
            frameSource = GetComponentInParent<XrMovableWindow>()
                ?.GetComponentInChildren<PilotSharedFrameTexture>(true);
            frameSource ??= FindFirstObjectByType<PilotSharedFrameTexture>();
        }

        // Canvas의 기존 글꼴을 우선 재사용한다.
        private static Font ResolveFont(RectTransform canvasRect)
        {
            Text existingText = canvasRect.GetComponentInChildren<Text>(true);
            return existingText != null && existingText.font != null
                ? existingText.font
                : Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        }

        // 지정한 panel 비율에 비상호작용 Text를 만든다.
        private static Text CreateText(
            RectTransform parent,
            string objectName,
            Vector2 anchorMin,
            Vector2 anchorMax,
            int fontSize,
            FontStyle fontStyle,
            Font font)
        {
            GameObject textObject = new GameObject(
                objectName,
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Text));
            textObject.layer = parent.gameObject.layer;
            RectTransform textRect = (RectTransform)textObject.transform;
            textRect.SetParent(parent, false);
            textRect.anchorMin = anchorMin;
            textRect.anchorMax = anchorMax;
            textRect.offsetMin = Vector2.zero;
            textRect.offsetMax = Vector2.zero;
            Text text = textObject.GetComponent<Text>();
            text.alignment = TextAnchor.MiddleCenter;
            text.font = font;
            text.fontSize = fontSize;
            text.fontStyle = fontStyle;
            text.raycastTarget = false;
            return text;
        }

        // 표시 단계에 해당하는 고정 HUD 색을 반환한다.
        private static Color GetColor(DepthDistanceDisplayLevel level)
        {
            return level switch
            {
                DepthDistanceDisplayLevel.Clear => ClearColor,
                DepthDistanceDisplayLevel.Caution => CautionColor,
                DepthDistanceDisplayLevel.Danger => DangerColor,
                _ => UnavailableColor
            };
        }

        // 전체 상태 caption에 사용할 짧은 한글 라벨을 반환한다.
        private static string GetStatusLabel(DepthDistanceDisplayLevel level)
        {
            return level switch
            {
                DepthDistanceDisplayLevel.Clear => "여유",
                DepthDistanceDisplayLevel.Caution => "주의",
                DepthDistanceDisplayLevel.Danger => "근접",
                _ => "신뢰 부족"
            };
        }
    }
}
