using System;

namespace StrikerPC.Control
{
    [Serializable]
    public readonly struct ControlTimingSummary
    {
        public readonly int InputSampleCount;
        public readonly int JitterSampleCount;
        public readonly double InputToSendP95Milliseconds;
        public readonly double ScheduleJitterP95Milliseconds;
        public readonly double SendFailurePercent;

        // A/B 판정에 필요한 통계값을 보관한다.
        public ControlTimingSummary(int inputCount, int jitterCount, double inputP95, double jitterP95, double failurePercent)
        {
            InputSampleCount = inputCount;
            JitterSampleCount = jitterCount;
            InputToSendP95Milliseconds = inputP95;
            ScheduleJitterP95Milliseconds = jitterP95;
            SendFailurePercent = failurePercent;
        }

        // 통계값을 로그용 한 줄로 변환한다.
        public override string ToString()
        {
            return $"input p95={InputToSendP95Milliseconds:F3}ms ({InputSampleCount}), "
                + $"jitter p95={ScheduleJitterP95Milliseconds:F3}ms ({JitterSampleCount}), "
                + $"failure={SendFailurePercent:F4}%";
        }
    }

    public sealed class ControlTimingWindow
    {
        private const int Capacity = 24000;
        private readonly object sync = new object();
        private readonly long[] inputMicroseconds = new long[Capacity];
        private readonly long[] jitterMicroseconds = new long[Capacity];
        private int inputCount;
        private int jitterCount;
        private int inputWriteIndex;
        private int jitterWriteIndex;

        // 입력에서 UDP 송신까지 걸린 시간을 기록한다.
        public void AddInput(long microseconds)
        {
            lock (sync)
            {
                inputMicroseconds[inputWriteIndex] = Math.Max(0, microseconds);
                inputWriteIndex = (inputWriteIndex + 1) % Capacity;
                inputCount = Math.Min(inputCount + 1, Capacity);
            }
        }

        // 설정된 고정 주기의 송신 스케줄 지터를 기록한다.
        public void AddJitter(long microseconds)
        {
            lock (sync)
            {
                jitterMicroseconds[jitterWriteIndex] = Math.Max(0, microseconds);
                jitterWriteIndex = (jitterWriteIndex + 1) % Capacity;
                jitterCount = Math.Min(jitterCount + 1, Capacity);
            }
        }

        // 현재 표본의 p95와 송신 실패율을 계산한다.
        public ControlTimingSummary Capture(long sentFrames, long failedFrames)
        {
            lock (sync)
            {
                long total = sentFrames + failedFrames;
                double failurePercent = total > 0 ? failedFrames * 100.0 / total : 0.0;
                return new ControlTimingSummary(
                    inputCount,
                    jitterCount,
                    Percentile95(inputMicroseconds, inputCount) / 1000.0,
                    Percentile95(jitterMicroseconds, jitterCount) / 1000.0,
                    failurePercent);
            }
        }

        // 표본 창과 인덱스를 초기화한다.
        public void Reset()
        {
            lock (sync)
            {
                inputCount = 0;
                jitterCount = 0;
                inputWriteIndex = 0;
                jitterWriteIndex = 0;
            }
        }

        // 복사한 표본에서 95백분위 값을 구한다.
        private static long Percentile95(long[] source, int count)
        {
            if (count <= 0)
            {
                return 0;
            }

            long[] copy = new long[count];
            Array.Copy(source, copy, count);
            Array.Sort(copy);
            int index = Math.Min(count - 1, (int)Math.Ceiling(count * 0.95) - 1);
            return copy[index];
        }
    }
}
