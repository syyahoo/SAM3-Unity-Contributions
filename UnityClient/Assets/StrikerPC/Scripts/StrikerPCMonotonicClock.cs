using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace StrikerPC
{
    public static class StrikerPCMonotonicClock
    {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        private static readonly long WindowsFrequency = ReadWindowsFrequency();

        [DllImport("kernel32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool QueryPerformanceCounter(out long value);

        [DllImport("kernel32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool QueryPerformanceFrequency(out long value);
#endif

        // Python time.perf_counter_ns와 같은 시스템 QPC 기준의 나노초를 반환한다.
        public static ulong NowNanoseconds()
        {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
            if (WindowsFrequency > 0
                && QueryPerformanceCounter(out long counter)
                && counter >= 0)
            {
                return (ulong)Math.Round(
                    counter * (1_000_000_000d / WindowsFrequency));
            }
#endif
            return (ulong)Math.Round(
                Stopwatch.GetTimestamp() * (1_000_000_000d / Stopwatch.Frequency));
        }

        // 미래 시각은 0으로 고정하고 정상 source timestamp의 경과 시간을 밀리초로 계산한다.
        public static double CalculateAgeMilliseconds(ulong sourceTimestampNs)
        {
            if (sourceTimestampNs == 0UL)
            {
                return -1d;
            }

            ulong nowNanoseconds = NowNanoseconds();
            return nowNanoseconds > sourceTimestampNs
                ? (nowNanoseconds - sourceTimestampNs) / 1_000_000d
                : 0d;
        }

#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        // QPC 주파수는 프로세스 수명 동안 변하지 않으므로 한 번만 확인한다.
        private static long ReadWindowsFrequency()
        {
            return QueryPerformanceFrequency(out long frequency) && frequency > 0
                ? frequency
                : 0L;
        }
#endif
    }
}
