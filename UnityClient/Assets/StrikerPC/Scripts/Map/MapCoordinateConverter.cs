using UnityEngine;

namespace StrikerPC.Map
{
    public static class MapCoordinateConverter
    {
        public const string VenueXzMeters = "venue-xz-meters";
        public const string NormalizedTopLeft = "normalized-top-left";

        // 좌상단 기준 표시 좌표를 미터 단위 지도 좌표로 변환한다.
        public static Vector2 NormalizedToMap(VenueMapDefinition map, float normalizedX, float normalizedY)
        {
            float mapX = map.minX + Mathf.Clamp01(normalizedX) * map.Width;
            float mapZ = map.maxZ - Mathf.Clamp01(normalizedY) * map.Depth;
            return new Vector2(mapX, mapZ);
        }

        // 미터 단위 지도 좌표를 좌상단 기준 표시 좌표로 변환한다.
        public static Vector2 MapToNormalized(VenueMapDefinition map, float mapX, float mapZ)
        {
            float normalizedX = Mathf.InverseLerp(map.minX, map.maxX, mapX);
            float normalizedY = Mathf.InverseLerp(map.maxZ, map.minZ, mapZ);
            return new Vector2(normalizedX, normalizedY);
        }

        // 지도 미터 좌표를 정합된 Unity 월드 좌표로 변환한다.
        public static Vector3 MapToWorld(VenueMapDefinition map, Transform mapRoot, float mapX, float mapZ)
        {
            return MapToWorld(map, mapRoot, mapX, 0f, mapZ);
        }

        // 높이를 보존하면서 지도 정렬을 적용해 Unity 월드 좌표로 변환한다.
        public static Vector3 MapToWorld(
            VenueMapDefinition map,
            Transform mapRoot,
            float mapX,
            float mapY,
            float mapZ)
        {
            Vector3 offsetPoint = new Vector3(
                mapX + map.originOffsetX,
                mapY,
                mapZ + map.originOffsetZ);
            Vector3 alignedPoint = Quaternion.Euler(0f, map.yawDegrees, 0f) * offsetPoint;
            return mapRoot.TransformPoint(alignedPoint);
        }

        // Unity 월드 좌표를 지도 미터 좌표로 역변환한다.
        public static Vector2 WorldToMap(VenueMapDefinition map, Transform mapRoot, Vector3 worldPoint)
        {
            Vector3 local = mapRoot.InverseTransformPoint(worldPoint);
            Vector3 unrotated = Quaternion.Euler(0f, -map.yawDegrees, 0f) * local;
            return new Vector2(unrotated.x - map.originOffsetX, unrotated.z - map.originOffsetZ);
        }
    }
}
