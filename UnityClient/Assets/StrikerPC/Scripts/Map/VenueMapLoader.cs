using System;
using System.IO;
using UnityEngine;

namespace StrikerPC.Map
{
    public sealed class VenueMapLoader : MonoBehaviour
    {
        [SerializeField] private string relativePath = "Maps/venue-map.json";
        [SerializeField] private bool loadOnAwake = true;

        public VenueMapDefinition Current { get; private set; }
        public event Action<VenueMapDefinition> Loaded;

        // PCVR 시작 시 기준 지도 JSON을 읽는다.
        private void Awake()
        {
            if (loadOnAwake)
            {
                Load();
            }
        }

        // StreamingAssets의 측량 지도를 역직렬화한다.
        [ContextMenu("Map/Reload")]
        public bool Load()
        {
            string path = Path.Combine(Application.streamingAssetsPath, relativePath);
            try
            {
                VenueMapDefinition definition = JsonUtility.FromJson<VenueMapDefinition>(File.ReadAllText(path));
                if (definition == null || !definition.IsValid())
                {
                    Debug.LogError($"[Venue Map] 유효하지 않은 지도입니다: {path}", this);
                    return false;
                }

                Current = definition;
                Loaded?.Invoke(Current);
                return true;
            }
            catch (Exception exception)
            {
                Debug.LogError($"[Venue Map] 지도를 읽지 못했습니다: {exception.Message}", this);
                return false;
            }
        }
    }
}
