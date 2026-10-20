using System;
using UnityEngine;

namespace StrikerPC.Video
{
    public readonly struct DepthDistanceSample
    {
        public DepthDistanceSample(
            long sequence,
            long frameId,
            ulong sourceTimestampNs,
            float leftMeters,
            float centerMeters,
            float rightMeters,
            float coverage,
            float inferenceMilliseconds)
        {
            Sequence = sequence;
            FrameId = frameId;
            SourceTimestampNs = sourceTimestampNs;
            LeftMeters = leftMeters;
            CenterMeters = centerMeters;
            RightMeters = rightMeters;
            Coverage = coverage;
            InferenceMilliseconds = inferenceMilliseconds;
        }

        public long Sequence { get; }
        public long FrameId { get; }
        public ulong SourceTimestampNs { get; }
        public float LeftMeters { get; }
        public float CenterMeters { get; }
        public float RightMeters { get; }
        public float Coverage { get; }
        public float InferenceMilliseconds { get; }
    }

    public static class DepthDistanceProtocol
    {
        public const string Source = "depth-anything-v2-metric-indoor-small";
        public const float MinimumCoverage = 0.60f;

        [Serializable]
        private sealed class Message
        {
            public int version;
            public long sequence;
            public long frameId;
            public ulong sourceTimestampNs;
            public float leftMeters;
            public float centerMeters;
            public float rightMeters;
            public float coverage;
            public float inferenceMs;
            public string source;
        }

        // localhost JSON의 버전, 단위, coverage와 모델 출처를 검증한다.
        public static bool TryParse(string json, out DepthDistanceSample sample)
        {
            sample = default;
            Message message;
            try
            {
                message = JsonUtility.FromJson<Message>(json);
            }
            catch (ArgumentException)
            {
                return false;
            }

            if (message == null
                || message.version != 1
                || message.sequence <= 0
                || message.frameId <= 0
                || message.sourceTimestampNs == 0UL
                || !string.Equals(message.source, Source, StringComparison.Ordinal)
                || !IsDistance(message.leftMeters)
                || !IsDistance(message.centerMeters)
                || !IsDistance(message.rightMeters)
                || !IsFinite(message.coverage)
                || message.coverage < MinimumCoverage
                || message.coverage > 1f
                || !IsFinite(message.inferenceMs)
                || message.inferenceMs < 0f)
            {
                return false;
            }

            sample = new DepthDistanceSample(
                message.sequence,
                message.frameId,
                message.sourceTimestampNs,
                message.leftMeters,
                message.centerMeters,
                message.rightMeters,
                message.coverage,
                message.inferenceMs);
            return true;
        }

        // HUD에 허용할 양수 유한 거리 범위인지 확인한다.
        private static bool IsDistance(float value)
        {
            return IsFinite(value) && value > 0f && value <= 100f;
        }

        // float 값이 NaN 또는 Infinity가 아닌지 확인한다.
        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
