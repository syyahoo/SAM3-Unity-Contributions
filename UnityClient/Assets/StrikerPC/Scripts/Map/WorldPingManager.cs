using System;
using System.Collections.Generic;
using UnityEngine;

namespace StrikerPC.Map
{
    public sealed class WorldPingManager : MonoBehaviour
    {
        [SerializeField] private VenueMapLoader mapLoader;
        [SerializeField] private Transform mapRoot;
        [SerializeField] private VenueSurfaceResolver surfaceResolver;
        [SerializeField] private bool createWorldVisuals;
        [SerializeField, Min(0.1f)] private float beamHeight = 2f;
        [SerializeField] private GameObject pingPrefab;
        [SerializeField] private string localPilotIdentity = "quest-pilot";
        [SerializeField] private Color localPilotColor = new Color(0.153f, 0.827f, 1f, 1f);
        [SerializeField] private Color adminColor = new Color(1f, 0.69f, 0.125f, 1f);
        [SerializeField] private Color remoteColor = new Color(1f, 0.302f, 0.427f, 1f);

        private readonly Dictionary<string, GameObject> activePings = new Dictionary<string, GameObject>();
        private readonly Dictionary<string, WorldPingEvent> activeAnchors = new Dictionary<string, WorldPingEvent>();
        private readonly Stack<GameObject> pingPool = new Stack<GameObject>();
        private VenueMapDefinition map;

        public VenueMapDefinition Map => map;
        public int ActivePingCount => activeAnchors.Count;
        public IReadOnlyDictionary<string, WorldPingEvent> ActiveAnchors => activeAnchors;
        public event Action<WorldPingEvent> AnchorCreated;
        public event Action<string> AnchorRemoved;
        public event Action AnchorsCleared;

        // 지도 로더와 전시장 좌표 기준을 연결한다.
        private void OnEnable()
        {
            if (mapLoader != null)
            {
                mapLoader.Loaded += OnMapLoaded;
                map = mapLoader.Current;
            }

            ValidateMapRoot();
        }

        // 지도 로더 이벤트 연결을 해제한다.
        private void OnDisable()
        {
            if (mapLoader != null)
            {
                mapLoader.Loaded -= OnMapLoaded;
            }
        }

        // 지도 좌표의 바닥 높이를 floor grid에서 확정해 앵커를 생성한다.
        public bool TryCreatePing(MapPingRequest request, out WorldPingEvent pingEvent, out string reason)
        {
            pingEvent = null;
            reason = string.Empty;
            if (map == null || mapRoot == null || surfaceResolver == null)
            {
                reason = "지도, MapRoot 또는 VenueSurfaceResolver가 준비되지 않았습니다.";
                return false;
            }

            if (request.mapId != map.mapId
                || request.mapVersion != map.version
                || (!string.IsNullOrWhiteSpace(request.scanId) && request.scanId != map.scanId))
            {
                reason = "스캔 ID 또는 지도 버전이 일치하지 않습니다.";
                return false;
            }

            if (!map.Contains(request.mapX, request.mapZ))
            {
                reason = "지도 경계를 벗어난 좌표입니다.";
                return false;
            }

            if (!surfaceResolver.TryResolve(request.mapX, request.mapZ, out float floorY))
            {
                reason = "스캔 floor grid에 유효한 바닥 높이가 없습니다.";
                return false;
            }

            Vector3 venuePoint = new Vector3(request.mapX, floorY, request.mapZ);
            Vector3 worldPoint = MapCoordinateConverter.MapToWorld(
                map,
                mapRoot,
                venuePoint.x,
                venuePoint.y,
                venuePoint.z);
            string id = string.IsNullOrWhiteSpace(request.requestId)
                ? Guid.NewGuid().ToString("N")
                : request.requestId;
            GameObject instance = null;
            if (createWorldVisuals && activePings.TryGetValue(id, out GameObject previous))
            {
                instance = previous;
                previous.transform.position = worldPoint;
            }
            else if (createWorldVisuals)
            {
                instance = AcquirePing();
                instance.name = $"WorldAnchor-{id}";
                instance.transform.SetPositionAndRotation(worldPoint, Quaternion.identity);
                instance.SetActive(true);
                activePings.Add(id, instance);
            }
            else if (activePings.Remove(id, out GameObject staleVisual))
            {
                staleVisual.SetActive(false);
                pingPool.Push(staleVisual);
            }

            pingEvent = new WorldPingEvent
            {
                id = id,
                requestId = request.requestId,
                mapId = map.mapId,
                mapVersion = map.version,
                scanId = map.scanId,
                mapX = request.mapX,
                mapZ = request.mapZ,
                venueX = venuePoint.x,
                venueY = venuePoint.y,
                venueZ = venuePoint.z,
                worldX = worldPoint.x,
                worldY = worldPoint.y,
                worldZ = worldPoint.z,
                requesterIdentity = request.requesterIdentity,
                requesterRole = request.requesterRole,
                timestamp = request.timestamp
            };
            if (instance != null)
            {
                ApplyVisualColor(instance, pingEvent);
            }
            activeAnchors[id] = pingEvent;
            AnchorCreated?.Invoke(pingEvent);
            return true;
        }

        // 수신 스냅샷을 현재 지도와 floor grid 기준으로 다시 투영한다.
        public void ReplaceFromSnapshot(IEnumerable<WorldPingEvent> snapshot)
        {
            ClearAll();
            if (snapshot == null)
            {
                return;
            }

            foreach (WorldPingEvent item in snapshot)
            {
                TryRestorePing(item, out _);
            }
        }

        // 확정된 단일 핑 이벤트를 현재 지도 좌표계에 반영한다.
        public bool TryRestorePing(WorldPingEvent item, out string reason)
        {
            reason = "잘못된 핑 이벤트입니다.";
            if (item == null || string.IsNullOrWhiteSpace(item.id))
            {
                return false;
            }

            MapPingRequest request = new MapPingRequest
            {
                type = "map-ping-request",
                requestId = item.id,
                mapId = item.mapId,
                mapVersion = item.mapVersion,
                scanId = item.scanId,
                mapX = item.mapX,
                mapZ = item.mapZ,
                requesterIdentity = item.requesterIdentity,
                requesterRole = item.requesterRole,
                timestamp = item.timestamp
            };
            return TryCreatePing(request, out _, out reason);
        }

        // 현재 파일럿이 소유한 핑인지 확인한다.
        public bool IsOwnedByLocalPilot(string id)
        {
            return !string.IsNullOrWhiteSpace(id)
                && activeAnchors.TryGetValue(id, out WorldPingEvent item)
                && item.requesterIdentity == localPilotIdentity;
        }

        // 지정한 화면 핑 데이터를 제거하고 남은 월드 표시가 있으면 풀로 반환한다.
        public bool RemovePing(string id)
        {
            if (string.IsNullOrWhiteSpace(id) || !activeAnchors.Remove(id))
            {
                return false;
            }

            if (activePings.Remove(id, out GameObject instance))
            {
                instance.SetActive(false);
                pingPool.Push(instance);
            }
            AnchorRemoved?.Invoke(id);
            return true;
        }

        // 모든 전시장 앵커를 제거하고 풀로 반환한다.
        public void ClearAll()
        {
            foreach (GameObject instance in activePings.Values)
            {
                instance.SetActive(false);
                pingPool.Push(instance);
            }

            activePings.Clear();
            activeAnchors.Clear();
            AnchorsCleared?.Invoke();
        }

        // 새 스캔 지도가 로드되면 기존 앵커를 비운다.
        private void OnMapLoaded(VenueMapDefinition definition)
        {
            ClearAll();
            map = definition;
        }

        // 풀에서 표시 오브젝트를 가져오거나 새로 만든다.
        private GameObject AcquirePing()
        {
            if (pingPool.Count > 0)
            {
                return pingPool.Pop();
            }

            GameObject instance = pingPrefab != null
                ? Instantiate(pingPrefab, transform, false)
                : CreateFallbackPing();
            ConfigureBeam(instance);
            return instance;
        }

        // 핑 소유자 역할을 지도 UI와 동일한 색상으로 표시한다.
        private void ApplyVisualColor(GameObject instance, WorldPingEvent pingEvent)
        {
            Color color = pingEvent.requesterIdentity == localPilotIdentity
                ? localPilotColor
                : pingEvent.requesterRole == "admin"
                    ? adminColor
                    : remoteColor;
            Renderer[] renderers = instance.GetComponentsInChildren<Renderer>(true);
            foreach (Renderer item in renderers)
            {
                MaterialPropertyBlock block = new MaterialPropertyBlock();
                item.GetPropertyBlock(block);
                block.SetColor("_BaseColor", color);
                block.SetColor("_Color", color);
                item.SetPropertyBlock(block);
            }
        }

        // Prefab의 Beam 자식 높이를 설정한다.
        private void ConfigureBeam(GameObject instance)
        {
            Transform beam = instance.transform.Find("Beam");
            if (beam == null)
            {
                return;
            }

            Vector3 scale = beam.localScale;
            beam.localScale = new Vector3(scale.x, beamHeight * 0.5f, scale.z);
            beam.localPosition = new Vector3(0f, beamHeight * 0.5f, 0f);
        }

        // Prefab이 없을 때 최소 앵커 표시를 생성한다.
        private GameObject CreateFallbackPing()
        {
            GameObject root = new GameObject("WorldAnchor");
            root.transform.SetParent(transform, false);

            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Marker";
            marker.transform.SetParent(root.transform, false);
            marker.transform.localPosition = new Vector3(0f, 0.12f, 0f);
            marker.transform.localScale = Vector3.one * 0.24f;
            RemoveCollider(marker);

            GameObject beam = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            beam.name = "Beam";
            beam.transform.SetParent(root.transform, false);
            beam.transform.localScale = new Vector3(0.04f, beamHeight * 0.5f, 0.04f);
            beam.transform.localPosition = new Vector3(0f, beamHeight * 0.5f, 0f);
            RemoveCollider(beam);
            return root;
        }

        // 표시 전용 오브젝트의 Collider를 제거한다.
        private static void RemoveCollider(GameObject target)
        {
            Collider collider = target.GetComponent<Collider>();
            if (collider != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(collider);
                }
                else
                {
                    DestroyImmediate(collider);
                }
            }
        }

        // MapRoot가 위치와 Yaw만 사용하는지 확인한다.
        private void ValidateMapRoot()
        {
            if (mapRoot == null)
            {
                return;
            }

            Vector3 angles = mapRoot.eulerAngles;
            if (Mathf.Abs(Mathf.DeltaAngle(angles.x, 0f)) > 0.01f
                || Mathf.Abs(Mathf.DeltaAngle(angles.z, 0f)) > 0.01f)
            {
                Debug.LogError("[Venue Map] MapRoot는 위치와 Yaw만 사용해야 합니다.", mapRoot);
            }
        }
    }
}
