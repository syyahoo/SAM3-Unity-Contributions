using System;
using System.IO;
using UnityEngine;

namespace StrikerPC.Map
{
    public sealed class VenueSurfaceResolver : MonoBehaviour
    {
        private const uint FloorMagic = 0x524C4656;
        private const int FloorVersion = 1;

        [SerializeField] private VenueMapLoader mapLoader;
        [SerializeField] private string fallbackRelativePath = "Maps/venue-floor.bin";

        private float[] heights;
        private int width;
        private int height;
        private float minX;
        private float minZ;
        private float cellSize;
        private float noDataValue;

        public bool IsLoaded => heights != null;

        // 지도 로더 이벤트에 바닥 높이 데이터 로드를 연결한다.
        private void OnEnable()
        {
            if (mapLoader != null)
            {
                mapLoader.Loaded += OnMapLoaded;
                if (mapLoader.Current != null)
                {
                    Load(mapLoader.Current);
                }
            }
        }

        // 지도 로더 이벤트 연결을 해제한다.
        private void OnDisable()
        {
            if (mapLoader != null)
            {
                mapLoader.Loaded -= OnMapLoaded;
            }
        }

        // 지도 X/Z에서 보간된 전시장 바닥 Y를 구한다.
        public bool TryResolve(float venueX, float venueZ, out float floorY)
        {
            floorY = 0f;
            if (heights == null || width < 1 || height < 1 || cellSize <= 0f)
            {
                return false;
            }

            float gridX = (venueX - minX) / cellSize;
            float gridZ = (venueZ - minZ) / cellSize;
            if (gridX < 0f || gridZ < 0f || gridX > width - 1 || gridZ > height - 1)
            {
                return false;
            }

            int x0 = Mathf.FloorToInt(gridX);
            int z0 = Mathf.FloorToInt(gridZ);
            int x1 = Mathf.Min(x0 + 1, width - 1);
            int z1 = Mathf.Min(z0 + 1, height - 1);
            float tx = gridX - x0;
            float tz = gridZ - z0;
            float h00 = heights[z0 * width + x0];
            float h10 = heights[z0 * width + x1];
            float h01 = heights[z1 * width + x0];
            float h11 = heights[z1 * width + x1];
            if (IsNoData(h00) || IsNoData(h10) || IsNoData(h01) || IsNoData(h11))
            {
                return false;
            }

            floorY = Mathf.Lerp(Mathf.Lerp(h00, h10, tx), Mathf.Lerp(h01, h11, tx), tz);
            return true;
        }

        // 새 지도 정의의 floor grid를 읽는다.
        private void OnMapLoaded(VenueMapDefinition definition)
        {
            Load(definition);
        }

        // 버전 고정 이진 floor grid를 검증하며 읽는다.
        private void Load(VenueMapDefinition definition)
        {
            heights = null;
            string relativePath = string.IsNullOrWhiteSpace(definition.floorDataPath)
                ? fallbackRelativePath
                : definition.floorDataPath;
            string path = Path.Combine(Application.streamingAssetsPath, relativePath);
            try
            {
                using FileStream stream = File.OpenRead(path);
                using BinaryReader reader = new BinaryReader(stream);
                uint magic = reader.ReadUInt32();
                int version = reader.ReadInt32();
                if (magic != FloorMagic || version != FloorVersion)
                {
                    throw new InvalidDataException("지원하지 않는 venue floor 형식입니다.");
                }

                width = reader.ReadInt32();
                height = reader.ReadInt32();
                minX = reader.ReadSingle();
                minZ = reader.ReadSingle();
                cellSize = reader.ReadSingle();
                noDataValue = reader.ReadSingle();
                int count = checked(width * height);
                if (width < 1 || height < 1 || count > 100_000_000)
                {
                    throw new InvalidDataException("venue floor 크기가 유효하지 않습니다.");
                }

                float[] loaded = new float[count];
                for (int index = 0; index < count; index++)
                {
                    loaded[index] = reader.ReadSingle();
                }
                heights = loaded;
            }
            catch (Exception exception)
            {
                Debug.LogError($"[Venue Surface] 바닥 높이 데이터를 읽지 못했습니다: {exception.Message}", this);
            }
        }

        // floor grid의 빈 셀인지 확인한다.
        private bool IsNoData(float value)
        {
            return float.IsNaN(value) || Mathf.Approximately(value, noDataValue);
        }
    }
}
