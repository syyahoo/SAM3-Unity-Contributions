using System;

namespace StrikerPC.Map
{
    [Serializable]
    public sealed class MapCommandHeader
    {
        public string type;
        public string id;
        public string requestId;
    }

    [Serializable]
    public sealed class MapPingRequest
    {
        public string type;
        public string requestId;
        public string mapId;
        public int mapVersion;
        public string scanId;
        public float mapX;
        public float mapZ;
        public string requesterIdentity;
        public string requesterRole;
        public long timestamp;
    }

    [Serializable]
    public sealed class WorldPingEvent
    {
        public string type = "ping-event";
        public string id;
        public string requestId;
        public string mapId;
        public int mapVersion;
        public string scanId;
        public float mapX;
        public float mapZ;
        public float worldX;
        public float worldY;
        public float worldZ;
        public float venueX;
        public float venueY;
        public float venueZ;
        public string requesterIdentity;
        public string requesterRole;
        public long timestamp;
    }

    [Serializable]
    public sealed class PingRejectedEvent
    {
        public string type = "ping-rejected";
        public string requestId;
        public string requesterIdentity;
        public string requesterRole;
        public string reason;
    }

    [Serializable]
    public sealed class PingDeletedEvent
    {
        public string type = "ping-deleted";
        public string id;
        public string requestId;
    }

    [Serializable]
    public sealed class MapStateMessage
    {
        public string type = "map-state";
        public string mapId;
        public int mapVersion;
        public string scanId;
        public string name;
        public string coordinateSpace;
        public float minX;
        public float maxX;
        public float minZ;
        public float maxZ;
        public float mapWidthMeters;
        public float mapHeightMeters;
        public int imageWidthPixels;
        public int imageHeightPixels;
        public float metersPerPixelX;
        public float metersPerPixelZ;
        public float imageOriginMapX;
        public float imageOriginMapZ;
        public string manifestUrl;
        public string imageUrl;
        public string manifestSha256;
        public string imageSha256;
    }

    [Serializable]
    public sealed class PingSnapshotMessage
    {
        public string type = "ping-snapshot";
        public string snapshotId;
        public int chunkIndex;
        public int chunkCount;
        public int revision;
        public WorldPingEvent[] pings;
    }

    [Serializable]
    public sealed class DroneMapTelemetry
    {
        public string type = "drone-telemetry";
        public string mapId;
        public int mapVersion;
        public string scanId;
        public float mapX;
        public float mapZ;
        public float venueY;
        public float yawDegrees;
        public float bearingDegrees;
        public string bearingCardinal;
        public string bearingReference;
        public float rotationX;
        public float rotationY;
        public float rotationZ;
        public float rotationW;
        public bool tracking;
        public string trackingState;
        public string navigationStatus;
        public float confidence;
        public float positionStdDevX;
        public float positionStdDevY;
        public float positionStdDevZ;
        public float yawStdDev;
        public long sourceTimestampNs;
        public bool stateConnected;
        public int batteryPercent;
        public int flightTimeSeconds;
        public float heightMeters;
        public float pitchDegrees;
        public float rollDegrees;
        public string controlPath;
        public float videoFrameAgeMilliseconds;
        public float poseAgeMilliseconds;
        public int visualAnchorCount;
        public bool localMapReady;
        public bool localMapDegraded;
        public bool relocalizing;
        public long timestamp;
    }

    [Serializable]
    public sealed class DroneHeadingTelemetry
    {
        public string type = "drone-heading";
        public string mapId;
        public int mapVersion;
        public string scanId;
        public int headingDegrees;
        public string headingReference;
        public long sourceTimestampNs;
        public long timestamp;
    }
}
