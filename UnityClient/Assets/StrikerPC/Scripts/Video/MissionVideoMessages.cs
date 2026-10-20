using System;

namespace StrikerPC.Video
{
    [Serializable]
    public sealed class MissionMessageHeader
    {
        public string type;
        public int schemaVersion;
    }

    [Serializable]
    public sealed class MissionDetectionBox
    {
        public int index;
        public int classId;
        public string className;
        public float confidence;
        public float x;
        public float y;
        public float width;
        public float height;
    }

    [Serializable]
    public sealed class MissionDetectionFrame
    {
        public string type;
        public int schemaVersion;
        public string missionId;
        public string sessionId;
        public string droneId;
        public string sourceIdentity;
        public string sourceTransport;
        public string sourceTrack;
        public long sequence;
        public long sourceTimestampMs;
        public long receivedAtMs;
        public long ttlMs;
        public string coordinateSpace;
        public string frameCorrelation;
        public MissionDetectionBox[] boxes;
    }

    [Serializable]
    public sealed class MissionTargetState
    {
        public string type;
        public int schemaVersion;
        public string missionId;
        public string sessionId;
        public string droneId;
        public string sourceIdentity;
        public string sourceTransport;
        public string sourceTrack;
        public long sequence;
        public long sourceFrameSequence;
        public long receivedAtMs;
        public string targetId;
        public bool active;
        public bool locked;
        public MissionDetectionBox box;
    }
}
