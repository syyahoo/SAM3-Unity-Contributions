using System;

namespace StrikerPC.Map
{
    [Serializable]
    public sealed class VenueMapPoint
    {
        public float x;
        public float z;
    }

    [Serializable]
    public sealed class VenueMapFeature
    {
        public string id;
        public string type;
        public string label;
        public VenueMapPoint[] points;
    }

    [Serializable]
    public sealed class VenueMapDefinition
    {
        public string mapId;
        public int version;
        public string scanId;
        public string name;
        public float minX;
        public float maxX;
        public float minZ;
        public float maxZ;
        public int imageWidth;
        public int imageHeight;
        public string imagePath;
        public string floorDataPath;
        public float floorCellSize;
        public int floorWidth;
        public int floorHeight;
        public float[] boardToVenue;
        public float originOffsetX;
        public float originOffsetZ;
        public float yawDegrees;
        public VenueMapFeature[] features;

        public float Width => maxX - minX;
        public float Depth => maxZ - minZ;

        // 지도 필수 범위가 유효한지 확인한다.
        public bool IsValid()
        {
            return !string.IsNullOrWhiteSpace(mapId)
                && !string.IsNullOrWhiteSpace(scanId)
                && version > 0
                && Width > 0f
                && Depth > 0f;
        }

        // 미터 좌표가 지도 경계 안인지 확인한다.
        public bool Contains(float mapX, float mapZ)
        {
            return mapX >= minX && mapX <= maxX && mapZ >= minZ && mapZ <= maxZ;
        }
    }
}
