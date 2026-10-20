using System;
using System.Collections.Generic;
using System.IO;
using StrikerPC.Video;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

namespace StrikerPC.Map
{
    public sealed class QuestMapPanel : MonoBehaviour, IPointerClickHandler
    {
        [SerializeField] private VenueMapLoader mapLoader;
        [SerializeField] private MapPingUdpBridge pingBridge;
        [SerializeField] private WorldPingManager pingManager;
        [SerializeField] private RawImage mapImage;
        [SerializeField] private bool allowRuntimeCanvasFallback;
        [SerializeField] private string requesterIdentity = "quest-pilot";
        [SerializeField] private DronePoseHistoryProvider poseHistory;
        [SerializeField, Range(2f, 15f)] private float positionUpdateRateHz = 10f;
        [SerializeField] private bool rotateMapWithDroneHeading = true;
        [SerializeField, Range(1f, 4f)] private float mapZoom = 1.6f;
        [SerializeField, Min(0f)] private float edgeIndicatorPadding = 28f;
        [SerializeField, Range(-180f, 180f)] private float droneHeadingOffsetDegrees = 180f;
        [SerializeField, Min(0.01f)] private float rotationSmoothingSeconds = 0.15f;
        [SerializeField] private Color aprilTagColor = new Color(0.2f, 1f, 0.35f, 1f);
        [SerializeField] private Color visualTrackingColor = new Color(0.1f, 0.8f, 1f, 1f);
        [SerializeField] private Color sensorPredictionColor = new Color(1f, 0.7f, 0.1f, 1f);
        [SerializeField] private Color uncertainColor = new Color(1f, 0.2f, 0.15f, 1f);
        [SerializeField] private Button createPingButton;
        [SerializeField] private Button deleteOwnPingButton;
        [SerializeField] private Color localPingColor = new Color(0.153f, 0.827f, 1f, 1f);
        [SerializeField] private Color adminPingColor = new Color(1f, 0.69f, 0.125f, 1f);
        [SerializeField] private Color remotePingColor = new Color(1f, 0.302f, 0.427f, 1f);
        [SerializeField] private Color pendingPingColor = new Color(0.55f, 0.58f, 0.62f, 1f);
        [SerializeField] private Color inactiveModeColor = new Color(0.28f, 0.31f, 0.36f, 1f);
        [SerializeField] private Color rejectedModeColor = new Color(1f, 0.2f, 0.15f, 1f);
        [SerializeField, Min(8f)] private float pingMarkerSize = 20f;
        [SerializeField, Min(8f)] private float deleteHitRadius = 30f;

        private Texture2D loadedTexture;
        [SerializeField] private RectTransform droneMarker;
        [SerializeField] private Image droneMarkerBody;
        [SerializeField] private Image droneHeadingLine;
        [SerializeField] private Text bearingText;
        private float nextPositionUpdateTime;
        private float displayedMapRotation;
        private RectTransform mapViewport;
        private Vector3 mapImageBaseScale = Vector3.one;
        private Vector2 mapPanOffset;
        private float overlayScale = 1f;
        private readonly Dictionary<string, RectTransform> pingMarkers =
            new Dictionary<string, RectTransform>();
        private readonly Dictionary<string, RectTransform> pendingPingMarkers =
            new Dictionary<string, RectTransform>();
        private readonly Dictionary<string, RectTransform> pingDirectionIndicators =
            new Dictionary<string, RectTransform>();
        private readonly Dictionary<string, RectTransform> pendingDirectionIndicators =
            new Dictionary<string, RectTransform>();
        private readonly Dictionary<string, Vector2> pendingPingPoints =
            new Dictionary<string, Vector2>();
        private readonly Stack<RectTransform> pingMarkerPool = new Stack<RectTransform>();
        private Text[] mapTexts = Array.Empty<Text>();
        private MapInteractionMode interactionMode;
        private float rejectionFeedbackUntil;

        private enum MapInteractionMode
        {
            View,
            CreatePing,
            DeleteOwnPing
        }

        // 런타임 bootstrap에서 기존 지도 컴포넌트를 연결한다.
        public void Configure(
            VenueMapLoader venueMapLoader,
            MapPingUdpBridge mapPingBridge,
            WorldPingManager worldPingManager)
        {
            UnsubscribePingEvents();
            if (isActiveAndEnabled && pingBridge != null)
            {
                pingBridge.CommandRejected -= OnCommandRejected;
            }
            mapLoader = venueMapLoader;
            pingBridge = mapPingBridge;
            pingManager = worldPingManager;
            SubscribePingEvents();
            if (isActiveAndEnabled && pingBridge != null)
            {
                pingBridge.CommandRejected -= OnCommandRejected;
                pingBridge.CommandRejected += OnCommandRejected;
            }
            RefreshPingMarkers();
        }

        // 런타임 bootstrap에서 전시장 좌표 pose 기록을 연결한다.
        public void ConfigurePoseSource(DronePoseHistoryProvider historyProvider)
        {
            poseHistory = historyProvider;
        }

        // 지도 회전 모드를 런타임에서 전환한다.
        public void SetHeadingUp(bool enabled)
        {
            rotateMapWithDroneHeading = enabled;
        }

        // Quest 지도용 World Space Canvas와 이미지를 준비한다.
        private void Awake()
        {
            if (mapImage == null)
            {
                if (!allowRuntimeCanvasFallback)
                {
                    Debug.LogError(
                        "[Quest Map] Scene에 직렬화된 MapImage 참조가 없습니다.",
                        this);
                    return;
                }

                mapImage = CreateMapCanvas();
            }

            ConfigureMapViewport();
            CreateDroneMarker();
            EnsureBearingText();
            FindToolbarReferences();
            CacheMapTexts();
            UpdateOverlayScale();
            UpdateModeButtonColors();
        }

        // 핑 이벤트와 지도 모드 버튼을 연결한다.
        private void OnEnable()
        {
            SubscribePingEvents();
            if (pingBridge != null)
            {
                pingBridge.CommandRejected += OnCommandRejected;
            }
            createPingButton?.onClick.AddListener(ToggleCreateMode);
            deleteOwnPingButton?.onClick.AddListener(ToggleDeleteMode);
        }

        // 비활성화 시 런타임 이벤트 연결을 해제한다.
        private void OnDisable()
        {
            UnsubscribePingEvents();
            if (pingBridge != null)
            {
                pingBridge.CommandRejected -= OnCommandRejected;
            }
            createPingButton?.onClick.RemoveListener(ToggleCreateMode);
            deleteOwnPingButton?.onClick.RemoveListener(ToggleDeleteMode);
        }

        // 로드된 지도 이미지를 지도창에 표시한다.
        private void Start()
        {
            VenueMapDefinition map = mapLoader?.Current;
            if (map == null || mapImage == null)
            {
                return;
            }

            string path = Path.Combine(
                Application.streamingAssetsPath,
                "Maps",
                Path.GetFileName(map.imagePath));
            try
            {
                loadedTexture = new Texture2D(2, 2, TextureFormat.RGBA32, false);
                if (!loadedTexture.LoadImage(File.ReadAllBytes(path)))
                {
                    throw new InvalidOperationException("이미지 디코딩 실패");
                }
                ApplyMapTexture(loadedTexture);
            }
            catch (Exception exception)
            {
                Debug.LogError($"[Quest Map] 지도 이미지를 읽지 못했습니다: {exception.Message}", this);
            }
        }

        // PNG 원본과 지도 좌표가 같은 상하 방향을 사용하도록 표시한다.
        private void ApplyMapTexture(Texture texture)
        {
            mapImage.texture = texture;
            mapImage.uvRect = new Rect(0f, 0f, 1f, 1f);
        }

        // 지도와 오버레이가 창 경계를 벗어나면 잘리도록 뷰포트를 준비한다.
        private void ConfigureMapViewport()
        {
            if (mapImage == null)
            {
                return;
            }

            mapViewport = mapImage.rectTransform.parent as RectTransform;
            mapImageBaseScale = mapImage.rectTransform.localScale;
            mapImage.rectTransform.localScale = mapImageBaseScale * Mathf.Max(1f, mapZoom);
            if (mapViewport != null && mapViewport.GetComponent<RectMask2D>() == null)
            {
                mapViewport.gameObject.AddComponent<RectMask2D>();
            }
        }

        // 설정된 주기로 현재 드론 위치와 추적 상태를 지도에 반영한다.
        private void Update()
        {
            if (Time.unscaledTime < nextPositionUpdateTime)
            {
                return;
            }

            nextPositionUpdateTime = Time.unscaledTime
                + 1f / Mathf.Max(2f, positionUpdateRateHz);
            UpdateOverlayScale();
            UpdateDroneOverlay();
            UpdatePingOverlays();
            UpdateModeButtonColors();
            HideMapTexts();
        }

        // 전시장 pose를 지도 이미지 좌표와 카메라 헤딩으로 변환한다.
        private void UpdateDroneOverlay()
        {
            VenueMapDefinition map = mapLoader?.Current;
            if (map == null
                || mapImage == null
                || droneMarker == null
                || (pingBridge != null && !pingBridge.IsMapVerified))
            {
                SetMarkerVisible(false);
                return;
            }

            DronePoseSampleV2 currentPose = default;
            bool hasCurrentPose = poseHistory != null
                && poseHistory.TryGetLatestPose(out currentPose);
            bool sourcePoseFresh = hasCurrentPose && poseHistory.SourcePoseFresh;

            if (!TrySelectDisplayPose(
                    map,
                    hasCurrentPose,
                    sourcePoseFresh,
                    currentPose,
                    out DronePoseSampleV2 pose))
            {
                SetMarkerVisible(false);
                return;
            }

            float displayHeadingRotation = GetDisplayHeadingRotation(
                pose.venueRotation.eulerAngles.y,
                droneHeadingOffsetDegrees);
            float targetMapRotation = rotateMapWithDroneHeading
                ? -displayHeadingRotation
                : 0f;
            float blend = 1f - Mathf.Exp(
                -(1f / Mathf.Max(2f, positionUpdateRateHz))
                / Mathf.Max(0.01f, rotationSmoothingSeconds));
            displayedMapRotation = Mathf.LerpAngle(
                displayedMapRotation,
                targetMapRotation,
                blend);
            float zoom = Mathf.Max(1f, mapZoom);
            mapImage.rectTransform.localScale = mapImageBaseScale * zoom;
            mapImage.rectTransform.localRotation = Quaternion.Euler(
                0f,
                0f,
                displayedMapRotation);

            Vector2 normalized = MapCoordinateConverter.MapToNormalized(
                map,
                pose.venuePosition.x,
                pose.venuePosition.z);
            Rect bounds = mapImage.rectTransform.rect;
            mapPanOffset = CalculateCenteredMapPan(
                bounds,
                normalized,
                displayedMapRotation,
                zoom);
            mapImage.rectTransform.anchoredPosition = mapPanOffset;
            droneMarker.anchoredPosition = Vector2.zero;
            droneMarker.localRotation = rotateMapWithDroneHeading
                ? Quaternion.identity
                : Quaternion.Euler(0f, 0f, displayHeadingRotation);
            float bearingDegrees = AprilTagBoardBearing.FromVenueRotation(
                pose.venueRotation,
                map.boardToVenue);
            Color trackingColor = GetTrackingColor(pose.trackingState);
            SetMarkerColor(trackingColor);
            UpdateBearingText(bearingDegrees, trackingColor);
            SetMarkerVisible(true);
        }

        // fresh 센서 pose를 포함한 현재 유효 좌표만 지도에 표시한다.
        private bool TrySelectDisplayPose(
            VenueMapDefinition map,
            bool hasCurrentPose,
            bool sourcePoseFresh,
            DronePoseSampleV2 currentPose,
            out DronePoseSampleV2 displayPose)
        {
            displayPose = default;
            if (!hasCurrentPose
                || !sourcePoseFresh
                || currentPose.trackingState == DroneTrackingStateV2.Initializing
                || currentPose.trackingState == DroneTrackingStateV2.Uncertain
                || !IsFinite(currentPose.venuePosition)
                || !IsFinite(currentPose.venueRotation)
                || !map.Contains(
                    currentPose.venuePosition.x,
                    currentPose.venuePosition.z))
            {
                return false;
            }

            displayPose = currentPose;
            return true;
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x)
                && IsFinite(value.y)
                && IsFinite(value.z)
                && IsFinite(value.w);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        // 좌우 반전 없는 지도에서 드론 전방 오프셋을 화면 회전값으로 변환한다.
        private static float GetDisplayHeadingRotation(float venueYawDegrees, float headingOffsetDegrees)
        {
            return Mathf.DeltaAngle(0f, -(venueYawDegrees + headingOffsetDegrees));
        }

        // 지도 좌표를 현재 드론 중심·헤딩 업·확대 상태의 화면 좌표로 변환한다.
        private Vector2 ProjectMapPosition(Rect bounds, Vector2 normalized)
        {
            return ProjectMapPoint(
                bounds,
                normalized,
                displayedMapRotation,
                Mathf.Max(1f, mapZoom),
                mapPanOffset);
        }

        // 지정한 지도 좌표가 뷰포트 중앙에 오도록 지도 이동량을 계산한다.
        private static Vector2 CalculateCenteredMapPan(
            Rect bounds,
            Vector2 normalized,
            float rotationDegrees,
            float zoom)
        {
            return -ProjectMapPoint(
                bounds,
                normalized,
                rotationDegrees,
                zoom,
                Vector2.zero);
        }

        // 정규화 지도 좌표에 확대·회전·이동을 적용한다.
        private static Vector2 ProjectMapPoint(
            Rect bounds,
            Vector2 normalized,
            float rotationDegrees,
            float zoom,
            Vector2 panOffset)
        {
            Vector2 local = new Vector2(
                Mathf.Lerp(bounds.xMin, bounds.xMax, normalized.x),
                Mathf.Lerp(bounds.yMax, bounds.yMin, normalized.y));
            Vector3 rotated = Quaternion.Euler(0f, 0f, rotationDegrees)
                * (Vector3)(local * Mathf.Max(1f, zoom));
            return new Vector2(rotated.x, rotated.y) + panOffset;
        }

        // 2배 XR 캔버스 기준으로 아이콘 배율만 계산한다.
        private static float CalculateOverlayScale(Vector2 viewportSize)
        {
            float shortestSide = Mathf.Min(
                Mathf.Abs(viewportSize.x),
                Mathf.Abs(viewportSize.y));
            return Mathf.Clamp(shortestSide / 600f, 0.65f, 1.4f);
        }

        // 창 크기에 맞춰 지도 아이콘의 도형 크기만 갱신하고 글자 크기는 유지한다.
        private void UpdateOverlayScale()
        {
            if (mapViewport == null)
            {
                return;
            }

            float nextScale = CalculateOverlayScale(mapViewport.rect.size);
            if (Mathf.Approximately(overlayScale, nextScale))
            {
                return;
            }

            overlayScale = nextScale;
            if (droneMarker != null)
            {
                droneMarker.sizeDelta = new Vector2(22f, 22f) * overlayScale;
            }
            if (droneHeadingLine != null)
            {
                RectTransform headingRect = droneHeadingLine.rectTransform;
                headingRect.anchoredPosition = new Vector2(0f, 18f * overlayScale);
                headingRect.sizeDelta = new Vector2(6f, 24f) * overlayScale;
            }
            ApplyPingMarkerScale(pingMarkers.Values);
            ApplyPingMarkerScale(pendingPingMarkers.Values);
            ApplyPingMarkerScale(pingMarkerPool);
        }

        // 확정·대기·풀링된 핑 도형에 같은 배율을 적용한다.
        private void ApplyPingMarkerScale(IEnumerable<RectTransform> markers)
        {
            foreach (RectTransform marker in markers)
            {
                if (marker != null)
                {
                    marker.sizeDelta = Vector2.one * pingMarkerSize * overlayScale;
                }
            }
        }

        // Quest 포인터 클릭을 동일한 map-ping-request 계약으로 변환한다.
        public void OnPointerClick(PointerEventData eventData)
        {
            VenueMapDefinition map = mapLoader?.Current;
            if (map == null
                || pingBridge == null
                || !pingBridge.IsMapVerified
                || mapImage == null
                || interactionMode == MapInteractionMode.View)
            {
                return;
            }

            RectTransform rect = mapImage.rectTransform;
            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(
                    rect,
                    eventData.position,
                    eventData.pressEventCamera,
                    out Vector2 local))
            {
                return;
            }

            Rect bounds = rect.rect;
            float normalizedX = Mathf.InverseLerp(bounds.xMin, bounds.xMax, local.x);
            float normalizedY = 1f - Mathf.InverseLerp(bounds.yMin, bounds.yMax, local.y);
            Vector2 mapPoint = MapCoordinateConverter.NormalizedToMap(
                map,
                normalizedX,
                normalizedY);
            if (interactionMode == MapInteractionMode.CreatePing)
            {
                if (pingBridge.TryCreateLocalPing(
                        mapPoint.x,
                        mapPoint.y,
                        requesterIdentity,
                        out string requestId))
                {
                    AddPendingPing(requestId, mapPoint);
                }
                return;
            }

            TryDeleteOwnedPingAt(eventData);
        }

        // 핑 추가 모드를 토글한다.
        private void ToggleCreateMode()
        {
            interactionMode = interactionMode == MapInteractionMode.CreatePing
                ? MapInteractionMode.View
                : MapInteractionMode.CreatePing;
            UpdateModeButtonColors();
        }

        // 내 핑 삭제 모드를 토글한다.
        private void ToggleDeleteMode()
        {
            interactionMode = interactionMode == MapInteractionMode.DeleteOwnPing
                ? MapInteractionMode.View
                : MapInteractionMode.DeleteOwnPing;
            UpdateModeButtonColors();
        }

        // 클릭 위치에서 가장 가까운 파일럿 소유 핑을 삭제 요청한다.
        private void TryDeleteOwnedPingAt(PointerEventData eventData)
        {
            if (pingManager == null || mapViewport == null
                || !RectTransformUtility.ScreenPointToLocalPointInRectangle(
                    mapViewport,
                    eventData.position,
                    eventData.pressEventCamera,
                    out Vector2 localPoint))
            {
                return;
            }

            string nearestId = null;
            float nearestDistance = deleteHitRadius;
            foreach (KeyValuePair<string, RectTransform> pair in pingMarkers)
            {
                if (!pingManager.IsOwnedByLocalPilot(pair.Key))
                {
                    continue;
                }

                float distance = Vector2.Distance(localPoint, pair.Value.anchoredPosition);
                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                    nearestId = pair.Key;
                }
            }

            if (nearestId != null)
            {
                pingBridge.TryDeleteLocalPing(nearestId, requesterIdentity);
            }
        }

        // 최소 World Space Canvas를 LeftWindowAnchor 아래에 만든다.
        private RawImage CreateMapCanvas()
        {
            GameObject canvasObject = new GameObject(
                "QuestMapCanvas",
                typeof(RectTransform),
                typeof(Canvas),
                typeof(TrackedDeviceGraphicRaycaster));
            canvasObject.transform.SetParent(transform, false);
            Canvas canvas = canvasObject.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            RectTransform canvasRect = canvasObject.GetComponent<RectTransform>();
            canvasRect.sizeDelta = new Vector2(900f, 700f);
            canvasRect.localScale = Vector3.one * 0.001f;

            GameObject imageObject = new GameObject(
                "MapImage",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(RawImage));
            imageObject.transform.SetParent(canvasObject.transform, false);
            RawImage image = imageObject.GetComponent<RawImage>();
            image.color = Color.white;
            image.raycastTarget = true;
            RectTransform imageRect = image.rectTransform;
            imageRect.anchorMin = Vector2.zero;
            imageRect.anchorMax = Vector2.one;
            imageRect.offsetMin = Vector2.zero;
            imageRect.offsetMax = Vector2.zero;
            return image;
        }

        // 지도 이미지 위에 드론 위치와 전방을 표시할 UI를 만든다.
        private void CreateDroneMarker()
        {
            if (mapImage == null || droneMarker != null)
            {
                return;
            }

            GameObject markerObject = new GameObject(
                "DroneMapMarker",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Image));
            markerObject.transform.SetParent(mapImage.transform.parent, false);
            droneMarker = markerObject.GetComponent<RectTransform>();
            droneMarker.anchorMin = new Vector2(0.5f, 0.5f);
            droneMarker.anchorMax = new Vector2(0.5f, 0.5f);
            droneMarker.sizeDelta = new Vector2(22f, 22f);
            droneMarkerBody = markerObject.GetComponent<Image>();
            droneMarkerBody.raycastTarget = false;

            GameObject headingObject = new GameObject(
                "Heading",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Image));
            headingObject.transform.SetParent(markerObject.transform, false);
            RectTransform headingRect = headingObject.GetComponent<RectTransform>();
            headingRect.anchorMin = new Vector2(0.5f, 0.5f);
            headingRect.anchorMax = new Vector2(0.5f, 0.5f);
            headingRect.anchoredPosition = new Vector2(0f, 18f);
            headingRect.sizeDelta = new Vector2(6f, 24f);
            droneHeadingLine = headingObject.GetComponent<Image>();
            droneHeadingLine.raycastTarget = false;
            SetMarkerVisible(false);
        }

        // 드론 마커 옆에 보드 기준 방위각을 표시할 Text를 준비한다.
        private void EnsureBearingText()
        {
            if (mapImage == null || droneMarker == null || bearingText != null)
            {
                return;
            }

            Transform parent = droneMarker.parent;
            Transform existing = parent.Find("DroneBearingLabel");
            if (existing != null)
            {
                bearingText = existing.GetComponent<Text>();
                if (bearingText != null)
                {
                    return;
                }
            }

            Text styleSource = parent.GetComponentInChildren<Text>(true);
            GameObject labelObject = new GameObject(
                "DroneBearingLabel",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Text),
                typeof(Outline));
            labelObject.transform.SetParent(parent, false);
            RectTransform labelRect = labelObject.GetComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0.5f, 0.5f);
            labelRect.anchorMax = new Vector2(0.5f, 0.5f);
            labelRect.sizeDelta = new Vector2(120f, 28f);
            bearingText = labelObject.GetComponent<Text>();
            bearingText.font = styleSource != null
                ? styleSource.font
                : Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            bearingText.fontSize = 20;
            bearingText.alignment = TextAnchor.MiddleCenter;
            bearingText.raycastTarget = false;
            Outline outline = labelObject.GetComponent<Outline>();
            outline.effectColor = new Color(0f, 0f, 0f, 0.9f);
            outline.effectDistance = new Vector2(1.5f, -1.5f);
            labelObject.transform.SetAsLastSibling();
            labelObject.SetActive(false);
        }

        // 보드 기준 N(0도) 방위각을 마커 위쪽에 화면 정방향으로 표시한다.
        private void UpdateBearingText(float bearingDegrees, Color color)
        {
            if (bearingText == null || droneMarker == null)
            {
                return;
            }

            bearingText.text = AprilTagBoardBearing.Format(bearingDegrees);
            bearingText.color = color;
            RectTransform labelRect = bearingText.rectTransform;
            labelRect.anchoredPosition = droneMarker.anchoredPosition
                + new Vector2(0f, 34f * overlayScale);
            labelRect.localRotation = Quaternion.identity;
        }

        // 추적 상태에 대응하는 표식 색상을 반환한다.
        private Color GetTrackingColor(DroneTrackingStateV2 state)
        {
            return state switch
            {
                DroneTrackingStateV2.AprilTagLocked => aprilTagColor,
                DroneTrackingStateV2.VisualTracked => visualTrackingColor,
                DroneTrackingStateV2.SensorPredicted => sensorPredictionColor,
                _ => uncertainColor
            };
        }

        // 드론 표식의 본체와 방향선 색상을 함께 변경한다.
        private void SetMarkerColor(Color color)
        {
            if (droneMarkerBody != null)
            {
                droneMarkerBody.color = color;
            }
            if (droneHeadingLine != null)
            {
                droneHeadingLine.color = color;
            }
        }

        // 유효한 pose가 있을 때만 드론 표식을 표시한다.
        private void SetMarkerVisible(bool visible)
        {
            if (droneMarker != null && droneMarker.gameObject.activeSelf != visible)
            {
                droneMarker.gameObject.SetActive(visible);
            }
            if (bearingText != null && bearingText.gameObject.activeSelf != visible)
            {
                bearingText.gameObject.SetActive(visible);
            }
        }

        // Scene에 직렬화되지 않은 경우 이름으로 지도 모드 버튼을 복구한다.
        private void FindToolbarReferences()
        {
            Button[] buttons = GetComponentsInChildren<Button>(true);
            foreach (Button button in buttons)
            {
                if (createPingButton == null && button.name == "CreatePingButton")
                {
                    createPingButton = button;
                }
                else if (deleteOwnPingButton == null && button.name == "DeleteOwnPingButton")
                {
                    deleteOwnPingButton = button;
                }
            }
        }

        // 미니맵 창 아래의 모든 텍스트를 한 번 수집해 숨긴다.
        private void CacheMapTexts()
        {
            mapTexts = GetComponentsInChildren<Text>(true);
            HideMapTexts();
        }

        // 런타임 표시 갱신이 텍스트를 다시 켜더라도 렌더링 전에 숨긴다.
        private void HideMapTexts()
        {
            foreach (Text mapText in mapTexts)
            {
                if (mapText == null || IsPingToolbarText(mapText))
                {
                    continue;
                }

                if (mapText.gameObject.activeSelf)
                {
                    mapText.gameObject.SetActive(false);
                }
            }
        }

        // 핑 조작 버튼의 라벨은 지도 장식 텍스트 숨김 대상에서 제외한다.
        private bool IsPingToolbarText(Text mapText)
        {
            return (createPingButton != null
                    && mapText.transform.IsChildOf(createPingButton.transform))
                || (deleteOwnPingButton != null
                    && mapText.transform.IsChildOf(deleteOwnPingButton.transform));
        }

        // WorldPingManager의 생성·삭제·초기화 이벤트를 연결한다.
        private void SubscribePingEvents()
        {
            if (pingManager == null)
            {
                return;
            }

            pingManager.AnchorCreated -= OnPingCreated;
            pingManager.AnchorRemoved -= OnPingRemoved;
            pingManager.AnchorsCleared -= OnPingsCleared;
            pingManager.AnchorCreated += OnPingCreated;
            pingManager.AnchorRemoved += OnPingRemoved;
            pingManager.AnchorsCleared += OnPingsCleared;
        }

        // WorldPingManager 이벤트 연결을 해제한다.
        private void UnsubscribePingEvents()
        {
            if (pingManager == null)
            {
                return;
            }

            pingManager.AnchorCreated -= OnPingCreated;
            pingManager.AnchorRemoved -= OnPingRemoved;
            pingManager.AnchorsCleared -= OnPingsCleared;
        }

        // 현재 확정 핑에 대응하는 지도 아이콘을 모두 준비한다.
        private void RefreshPingMarkers()
        {
            if (pingManager == null || mapImage == null)
            {
                return;
            }

            foreach (WorldPingEvent item in pingManager.ActiveAnchors.Values)
            {
                OnPingCreated(item);
            }
            UpdatePingOverlays();
        }

        // 새 핑의 지도 아이콘을 생성하거나 색상을 갱신한다.
        private void OnPingCreated(WorldPingEvent pingEvent)
        {
            if (pingEvent == null || string.IsNullOrWhiteSpace(pingEvent.id) || mapImage == null)
            {
                return;
            }

            RemovePendingPing(pingEvent.requestId ?? pingEvent.id);
            if (!pingMarkers.TryGetValue(pingEvent.id, out RectTransform marker))
            {
                marker = AcquirePingMarker();
                marker.name = $"MapPing-{pingEvent.id}";
                pingMarkers.Add(pingEvent.id, marker);
            }

            marker.GetComponent<Image>().color = GetPingColor(pingEvent);
            marker.gameObject.SetActive(true);
        }

        // 삭제된 핑 아이콘을 비활성화해 풀에 반환한다.
        private void OnPingRemoved(string id)
        {
            if (!pingMarkers.Remove(id, out RectTransform marker))
            {
                return;
            }

            RemoveDirectionIndicator(id, pingDirectionIndicators);
            marker.gameObject.SetActive(false);
            pingMarkerPool.Push(marker);
        }

        // 모든 지도 핑 아이콘을 풀로 반환한다.
        private void OnPingsCleared()
        {
            foreach (RectTransform marker in pingMarkers.Values)
            {
                marker.gameObject.SetActive(false);
                pingMarkerPool.Push(marker);
            }
            pingMarkers.Clear();
            DestroyDirectionIndicators(pingDirectionIndicators);
            foreach (RectTransform marker in pendingPingMarkers.Values)
            {
                marker.gameObject.SetActive(false);
                pingMarkerPool.Push(marker);
            }
            pendingPingMarkers.Clear();
            pendingPingPoints.Clear();
            DestroyDirectionIndicators(pendingDirectionIndicators);
        }

        // Gateway 승인 전 요청 위치를 회색 지도 아이콘으로 표시한다.
        private void AddPendingPing(string requestId, Vector2 mapPoint)
        {
            if (string.IsNullOrWhiteSpace(requestId) || mapImage == null)
            {
                return;
            }

            RectTransform marker = AcquirePingMarker();
            marker.name = $"PendingMapPing-{requestId}";
            marker.GetComponent<Image>().color = pendingPingColor;
            marker.gameObject.SetActive(true);
            pendingPingMarkers[requestId] = marker;
            pendingPingPoints[requestId] = mapPoint;
            UpdatePingOverlays();
        }

        // 승인 또는 거부된 대기 핑 아이콘을 풀로 반환한다.
        private void RemovePendingPing(string requestId)
        {
            if (string.IsNullOrWhiteSpace(requestId)
                || !pendingPingMarkers.Remove(requestId, out RectTransform marker))
            {
                return;
            }

            RemoveDirectionIndicator(requestId, pendingDirectionIndicators);
            pendingPingPoints.Remove(requestId);
            marker.gameObject.SetActive(false);
            pingMarkerPool.Push(marker);
        }

        // 지도 회전에 맞춰 모든 핑 위치를 갱신하고 아이콘은 정방향을 유지한다.
        private void UpdatePingOverlays()
        {
            VenueMapDefinition map = mapLoader?.Current;
            if (map == null
                || mapImage == null
                || pingManager == null
                || (pingBridge != null && !pingBridge.IsMapVerified))
            {
                foreach (RectTransform marker in pingMarkers.Values)
                {
                    marker.gameObject.SetActive(false);
                }
                foreach (RectTransform marker in pendingPingMarkers.Values)
                {
                    marker.gameObject.SetActive(false);
                }
                SetDirectionIndicatorsVisible(pingDirectionIndicators, false);
                SetDirectionIndicatorsVisible(pendingDirectionIndicators, false);
                return;
            }

            Rect bounds = mapImage.rectTransform.rect;
            foreach (KeyValuePair<string, WorldPingEvent> pair in pingManager.ActiveAnchors)
            {
                if (!pingMarkers.TryGetValue(pair.Key, out RectTransform marker))
                {
                    OnPingCreated(pair.Value);
                    marker = pingMarkers[pair.Key];
                }

                Vector2 normalized = MapCoordinateConverter.MapToNormalized(
                    map,
                    pair.Value.mapX,
                    pair.Value.mapZ);
                PlacePingMarker(
                    pair.Key,
                    marker,
                    ProjectMapPosition(bounds, normalized),
                    GetPingColor(pair.Value),
                    pingDirectionIndicators);
            }

            foreach (KeyValuePair<string, Vector2> pair in pendingPingPoints)
            {
                if (!pendingPingMarkers.TryGetValue(pair.Key, out RectTransform marker))
                {
                    continue;
                }

                Vector2 normalized = MapCoordinateConverter.MapToNormalized(
                    map,
                    pair.Value.x,
                    pair.Value.y);
                PlacePingMarker(
                    pair.Key,
                    marker,
                    ProjectMapPosition(bounds, normalized),
                    pendingPingColor,
                    pendingDirectionIndicators);
            }
        }

        // 화면 안 핑은 마커로 표시하고 화면 밖 핑은 경계 화살표로 대체한다.
        private void PlacePingMarker(
            string id,
            RectTransform marker,
            Vector2 position,
            Color color,
            Dictionary<string, RectTransform> indicators)
        {
            if (marker == null || mapViewport == null)
            {
                return;
            }

            Rect viewportBounds = mapViewport.rect;
            float padding = Mathf.Min(
                edgeIndicatorPadding,
                Mathf.Min(viewportBounds.width, viewportBounds.height) * 0.45f);
            Rect visibleBounds = Rect.MinMaxRect(
                viewportBounds.xMin + padding,
                viewportBounds.yMin + padding,
                viewportBounds.xMax - padding,
                viewportBounds.yMax - padding);
            bool visible = visibleBounds.Contains(position);
            marker.anchoredPosition = position;
            marker.localRotation = Quaternion.identity;
            marker.gameObject.SetActive(visible);

            if (visible)
            {
                if (indicators.TryGetValue(id, out RectTransform existing))
                {
                    existing.gameObject.SetActive(false);
                }
                return;
            }

            RectTransform indicator = GetOrCreateDirectionIndicator(id, indicators);
            Vector2 direction = position - visibleBounds.center;
            indicator.anchoredPosition = ProjectToRectEdge(visibleBounds, direction);
            indicator.localRotation = Quaternion.Euler(
                0f,
                0f,
                Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg - 90f);
            Text indicatorText = indicator.GetComponent<Text>();
            if (indicatorText != null)
            {
                indicatorText.color = color;
            }
            indicator.gameObject.SetActive(true);
            indicator.SetAsLastSibling();
        }

        // 중심에서 마킹 방향으로 뻗은 선과 뷰포트 경계의 교점을 구한다.
        private static Vector2 ProjectToRectEdge(Rect bounds, Vector2 direction)
        {
            if (direction.sqrMagnitude <= 0.0001f)
            {
                return bounds.center;
            }

            float halfWidth = bounds.width * 0.5f;
            float halfHeight = bounds.height * 0.5f;
            float xScale = Mathf.Abs(direction.x) > 0.0001f
                ? halfWidth / Mathf.Abs(direction.x)
                : float.PositiveInfinity;
            float yScale = Mathf.Abs(direction.y) > 0.0001f
                ? halfHeight / Mathf.Abs(direction.y)
                : float.PositiveInfinity;
            return bounds.center + direction * Mathf.Min(xScale, yScale);
        }

        // 화면 밖 핑별 고정 픽셀 화살표를 생성하거나 재사용한다.
        private RectTransform GetOrCreateDirectionIndicator(
            string id,
            Dictionary<string, RectTransform> indicators)
        {
            if (indicators.TryGetValue(id, out RectTransform indicator))
            {
                return indicator;
            }

            GameObject indicatorObject = new GameObject(
                $"MapPingDirection-{id}",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Text),
                typeof(Outline));
            indicatorObject.transform.SetParent(mapViewport, false);
            indicator = indicatorObject.GetComponent<RectTransform>();
            indicator.anchorMin = new Vector2(0.5f, 0.5f);
            indicator.anchorMax = new Vector2(0.5f, 0.5f);
            indicator.sizeDelta = new Vector2(34f, 34f);
            Text text = indicatorObject.GetComponent<Text>();
            text.text = "▲";
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = 22;
            text.resizeTextForBestFit = false;
            text.alignment = TextAnchor.MiddleCenter;
            text.raycastTarget = false;
            Outline outline = indicatorObject.GetComponent<Outline>();
            outline.effectColor = new Color(0f, 0f, 0f, 0.9f);
            outline.effectDistance = new Vector2(1.5f, -1.5f);
            indicators.Add(id, indicator);
            return indicator;
        }

        private static void SetDirectionIndicatorsVisible(
            Dictionary<string, RectTransform> indicators,
            bool visible)
        {
            foreach (RectTransform indicator in indicators.Values)
            {
                if (indicator != null)
                {
                    indicator.gameObject.SetActive(visible);
                }
            }
        }

        private static void RemoveDirectionIndicator(
            string id,
            Dictionary<string, RectTransform> indicators)
        {
            if (indicators.Remove(id, out RectTransform indicator) && indicator != null)
            {
                Destroy(indicator.gameObject);
            }
        }

        private static void DestroyDirectionIndicators(
            Dictionary<string, RectTransform> indicators)
        {
            foreach (RectTransform indicator in indicators.Values)
            {
                if (indicator != null)
                {
                    Destroy(indicator.gameObject);
                }
            }
            indicators.Clear();
        }

        // 재사용 가능한 지도 핑 Image를 가져오거나 새로 만든다.
        private RectTransform AcquirePingMarker()
        {
            if (pingMarkerPool.Count > 0)
            {
                return pingMarkerPool.Pop();
            }

            GameObject markerObject = new GameObject(
                "MapPing",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(Image));
            markerObject.transform.SetParent(mapImage.transform.parent, false);
            RectTransform marker = markerObject.GetComponent<RectTransform>();
            marker.anchorMin = new Vector2(0.5f, 0.5f);
            marker.anchorMax = new Vector2(0.5f, 0.5f);
            marker.sizeDelta = Vector2.one * pingMarkerSize * overlayScale;
            Image image = markerObject.GetComponent<Image>();
            image.raycastTarget = false;
            return marker;
        }

        // 내 핑·관리자 핑·기타 수신 핑의 색상을 구분한다.
        private Color GetPingColor(WorldPingEvent pingEvent)
        {
            if (pingEvent.requesterIdentity == requesterIdentity)
            {
                return localPingColor;
            }
            return pingEvent.requesterRole == "admin" ? adminPingColor : remotePingColor;
        }

        // 활성 모드와 최근 거부 상태를 버튼 색상으로 표시한다.
        private void UpdateModeButtonColors()
        {
            bool rejected = Time.unscaledTime < rejectionFeedbackUntil;
            if (createPingButton?.targetGraphic != null)
            {
                createPingButton.interactable = true;
                createPingButton.targetGraphic.color = rejected
                    ? rejectedModeColor
                    : interactionMode == MapInteractionMode.CreatePing
                        ? localPingColor
                        : inactiveModeColor;
            }
            if (deleteOwnPingButton?.targetGraphic != null)
            {
                deleteOwnPingButton.interactable = true;
                deleteOwnPingButton.targetGraphic.color = rejected
                    ? rejectedModeColor
                    : interactionMode == MapInteractionMode.DeleteOwnPing
                        ? localPingColor
                        : inactiveModeColor;
            }
        }

        // Gateway 거부를 짧은 버튼 색상 피드백과 Console 경고로 알린다.
        private void OnCommandRejected(string requestId, string reason)
        {
            RemovePendingPing(requestId);
            rejectionFeedbackUntil = Time.unscaledTime + 1f;
            Debug.LogWarning($"[Quest Map] {reason}", this);
            UpdateModeButtonColors();
        }

        // 런타임에 로드한 Texture를 정리한다.
        private void OnDestroy()
        {
            if (loadedTexture != null)
            {
                Destroy(loadedTexture);
            }
        }
    }
}
