using System;
using System.IO.MemoryMappedFiles;
using System.Reflection;
using System.Runtime.InteropServices;
using UnityEngine;

namespace StrikerPC.Video
{
    public readonly struct SharedFrameTimingSummary
    {
        public readonly int SampleCount;
        public readonly double FrameAgeP95Milliseconds;

        public SharedFrameTimingSummary(int sampleCount, double frameAgeP95Milliseconds)
        {
            SampleCount = sampleCount;
            FrameAgeP95Milliseconds = frameAgeP95Milliseconds;
        }

        public override string ToString()
        {
            return $"frame age p95={FrameAgeP95Milliseconds:F3}ms ({SampleCount})";
        }
    }

    public sealed class PilotSharedFrameTexture : MonoBehaviour
    {
        private const uint FrameMagic = 0x31564654;
        private const uint ProtocolVersionV1 = 1;
        private const uint ProtocolVersionV2 = 2;
        private const int HeaderSizeV1 = 32;
        private const int HeaderSizeV2 = 40;
        private const int TimingCapacity = 24000;

        [SerializeField] private string sharedMemoryName = "TelloPilotFrame_v1";
        [SerializeField] private bool preferVersion2 = true;
        [SerializeField] private string version2SharedMemoryName = "TelloPilotFrame_v2";
        [SerializeField] private Renderer targetRenderer;
        [Tooltip("같은 FPV Texture를 추가 화면에 복제할 때 사용합니다.")]
        [SerializeField] private Renderer[] additionalRenderers;
        [Tooltip("선택 사항입니다. Unity UI RawImage를 연결할 수 있습니다.")]
        [SerializeField] private UnityEngine.Object rawImageTarget;
        [SerializeField, Min(0.1f)] private float staleWarningSeconds = 1f;

        private MemoryMappedFile mappedFile;
        private MemoryMappedViewAccessor accessor;
        private Texture2D texture;
        private byte[] pixelBuffer;
        private long lastFrameId = -1;
        private float nextOpenAttemptTime;
        private MaterialPropertyBlock propertyBlock;
        private PropertyInfo rawImageTextureProperty;
        private PropertyInfo rawImageUvRectProperty;
        private int activeHeaderSize = HeaderSizeV1;
        private float lastFrameReceivedAt = -1f;
        private bool staleWarningLogged;
        private readonly long[] frameAgeMicroseconds = new long[TimingCapacity];
        private int frameAgeCount;
        private int frameAgeWriteIndex;

        public Texture2D Texture => texture;
        public bool IsConnected => accessor != null;
        public long LastFrameId => lastFrameId;
        public ulong LastSourceTimestampNs { get; private set; }
        public double LastFrameAgeMilliseconds { get; private set; }

        // 디코드 프레임 게시부터 Texture 반영까지의 p95를 반환한다.
        public SharedFrameTimingSummary CaptureTimingSummary()
        {
            int count = frameAgeCount;
            if (count <= 0)
            {
                return new SharedFrameTimingSummary(0, 0d);
            }

            long[] copy = new long[count];
            Array.Copy(frameAgeMicroseconds, copy, count);
            Array.Sort(copy);
            int index = Mathf.Min(count - 1, Mathf.CeilToInt(count * 0.95f) - 1);
            return new SharedFrameTimingSummary(count, copy[index] / 1000d);
        }

        // 프레임 지연 측정 표본을 초기화한다.
        [ContextMenu("Video/Reset Shared Frame Timing")]
        public void ResetTimingMetrics()
        {
            frameAgeCount = 0;
            frameAgeWriteIndex = 0;
            LastFrameAgeMilliseconds = 0d;
        }

        // 현재 프레임 지연 p95를 Console에 출력한다.
        [ContextMenu("Video/Log Shared Frame Timing")]
        public void LogTimingMetrics()
        {
            UnityEngine.Debug.Log($"[Shared FPV] {CaptureTimingSummary()}", this);
        }

        // 지정 시간 안에 새 공유 프레임을 실제 Texture에 반영했는지 확인한다.
        public bool HasFreshFrame(float maximumAgeSeconds)
        {
            return accessor != null
                && texture != null
                && lastFrameReceivedAt >= 0f
                && Time.unscaledTime - lastFrameReceivedAt <= Mathf.Max(0f, maximumAgeSeconds);
        }

        // 공유 메모리의 최신 프레임만 Texture에 반영한다.
        private void Update()
        {
            if (accessor == null && !TryOpenSharedMemory())
            {
                return;
            }

            try
            {
                TryApplyLatestFrame();
                if (ShouldLogStaleWarning(Time.unscaledTime))
                {
                    UnityEngine.Debug.LogWarning(
                        "[Shared FPV] 최신 SharedMemory 영상 프레임이 1초 이상 수신되지 않았습니다.",
                        this);
                }
            }
            catch (Exception)
            {
                CloseSharedMemory();
            }
        }

        // 기록 경합 시 최신 프레임을 한 번 더 읽어 Texture에 반영한다.
        private void TryApplyLatestFrame()
        {
            for (int attempt = 0; attempt < 2; attempt++)
            {
                uint magic = accessor.ReadUInt32(0);
                uint version = accessor.ReadUInt32(4);
                int width = accessor.ReadInt32(8);
                int height = accessor.ReadInt32(12);
                int stride = accessor.ReadInt32(16);
                int pixelFormat = accessor.ReadInt32(20);
                long frameId = accessor.ReadInt64(24);

                if (magic != FrameMagic
                    || (version != ProtocolVersionV1 && version != ProtocolVersionV2)
                    || pixelFormat != 1
                    || frameId <= 0 || width <= 0 || height <= 0 || stride != width * 4)
                {
                    return;
                }

                activeHeaderSize = version == ProtocolVersionV2 ? HeaderSizeV2 : HeaderSizeV1;
                ulong sourceTimestampNs = version == ProtocolVersionV2 ? accessor.ReadUInt64(32) : 0UL;
                if (frameId <= lastFrameId
                    && !IsWriterRestartFrame(
                        version,
                        frameId,
                        lastFrameId,
                        sourceTimestampNs,
                        LastSourceTimestampNs))
                {
                    return;
                }
                int byteCount = checked(stride * height);
                EnsureTexture(width, height, byteCount);
                CopyFramePixels(byteCount);
                if (accessor.ReadInt64(24) != frameId)
                {
                    continue;
                }

                texture.LoadRawTextureData(pixelBuffer);
                texture.Apply(false, false);
                lastFrameId = frameId;
                LastSourceTimestampNs = sourceTimestampNs;
                lastFrameReceivedAt = Time.unscaledTime;
                staleWarningLogged = false;
                RecordFrameAge(sourceTimestampNs);
                return;
            }
        }

        // 같은 Named Memory에서 writer가 재시작해 frame ID가 초기화된 경우를 판별한다.
        private static bool IsWriterRestartFrame(
            uint version,
            long frameId,
            long previousFrameId,
            ulong sourceTimestampNs,
            ulong previousSourceTimestampNs)
        {
            return version == ProtocolVersionV2
                && frameId > 0
                && frameId <= previousFrameId
                && sourceTimestampNs > previousSourceTimestampNs;
        }

        // 동일 PC의 monotonic clock으로 프레임 전달 지연을 기록한다.
        private void RecordFrameAge(ulong sourceTimestampNs)
        {
            if (sourceTimestampNs == 0UL)
            {
                return;
            }

            double ageMilliseconds = global::StrikerPC.StrikerPCMonotonicClock
                .CalculateAgeMilliseconds(sourceTimestampNs);
            long ageMicroseconds = (long)Math.Min(
                long.MaxValue,
                Math.Max(0d, ageMilliseconds * 1000d));
            LastFrameAgeMilliseconds = ageMicroseconds / 1000d;
            frameAgeMicroseconds[frameAgeWriteIndex] = ageMicroseconds;
            frameAgeWriteIndex = (frameAgeWriteIndex + 1) % TimingCapacity;
            frameAgeCount = Math.Min(frameAgeCount + 1, TimingCapacity);
        }

        // 한 번 이상 수신한 영상이 만료되면 복구 전까지 경고를 한 번만 허용한다.
        private bool ShouldLogStaleWarning(float now)
        {
            bool stale = lastFrameReceivedAt >= 0f
                && now - lastFrameReceivedAt >= staleWarningSeconds;
            if (!stale)
            {
                staleWarningLogged = false;
                return false;
            }

            if (staleWarningLogged)
            {
                return false;
            }

            staleWarningLogged = true;
            return true;
        }

        // 공유 메모리 픽셀을 관리 배열로 일괄 복사한다.
        private void CopyFramePixels(int byteCount)
        {
            var viewHandle = accessor.SafeMemoryMappedViewHandle;
            bool addedReference = false;
            try
            {
                viewHandle.DangerousAddRef(ref addedReference);
                long pixelOffset = checked(accessor.PointerOffset + activeHeaderSize);
                IntPtr source = new IntPtr(
                    checked(viewHandle.DangerousGetHandle().ToInt64() + pixelOffset));
                Marshal.Copy(source, pixelBuffer, 0, byteCount);
            }
            finally
            {
                if (addedReference)
                {
                    viewHandle.DangerousRelease();
                }
            }
        }

        // 공유 메모리와 Texture 자원을 정리한다.
        private void OnDisable()
        {
            CloseSharedMemory();
            if (texture != null)
            {
                Destroy(texture);
                texture = null;
            }
        }

        // VideoHub가 만든 이름 기반 공유 메모리를 연다.
        private bool TryOpenSharedMemory()
        {
            if (Time.unscaledTime < nextOpenAttemptTime)
            {
                return false;
            }

            nextOpenAttemptTime = Time.unscaledTime + 1f;
            try
            {
                if (preferVersion2 && TryOpenNamedMemory(version2SharedMemoryName))
                {
                    return true;
                }

                return TryOpenNamedMemory(sharedMemoryName);
            }
            catch (Exception)
            {
                CloseSharedMemory();
                return false;
            }
        }

        // 지정한 이름의 공유 메모리를 읽기 전용으로 연다.
        private bool TryOpenNamedMemory(string memoryName)
        {
            if (string.IsNullOrWhiteSpace(memoryName))
            {
                return false;
            }

            try
            {
                mappedFile = MemoryMappedFile.OpenExisting(memoryName, MemoryMappedFileRights.Read);
                accessor = mappedFile.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read);
                return true;
            }
            catch (Exception)
            {
                CloseSharedMemory();
                return false;
            }
        }

        // 해상도가 바뀌면 Texture와 재사용 버퍼를 다시 만든다.
        private void EnsureTexture(int width, int height, int byteCount)
        {
            if (texture != null && texture.width == width && texture.height == height
                && pixelBuffer?.Length == byteCount)
            {
                return;
            }

            if (texture != null)
            {
                Destroy(texture);
            }

            texture = new Texture2D(width, height, TextureFormat.BGRA32, false, false)
            {
                name = "Tello Pilot Frame",
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp
            };
            pixelBuffer = new byte[byteCount];
            ApplyTextureToTargets();
        }

        // 생성한 Texture를 Renderer 또는 RawImage에 연결한다.
        private void ApplyTextureToTargets()
        {
            if (targetRenderer != null)
            {
                ApplyTextureToRenderer(targetRenderer);
            }

            if (additionalRenderers != null)
            {
                foreach (Renderer additionalRenderer in additionalRenderers)
                {
                    ApplyTextureToRenderer(additionalRenderer);
                }
            }

            if (rawImageTarget == null)
            {
                return;
            }

            rawImageTextureProperty ??= rawImageTarget.GetType().GetProperty(
                "texture",
                BindingFlags.Instance | BindingFlags.Public);
            if (rawImageTextureProperty != null && rawImageTextureProperty.CanWrite
                && typeof(Texture).IsAssignableFrom(rawImageTextureProperty.PropertyType))
            {
                rawImageTextureProperty.SetValue(rawImageTarget, texture);
            }

            rawImageUvRectProperty ??= rawImageTarget.GetType().GetProperty(
                "uvRect",
                BindingFlags.Instance | BindingFlags.Public);
            if (rawImageUvRectProperty != null
                && rawImageUvRectProperty.CanWrite
                && rawImageUvRectProperty.PropertyType == typeof(Rect))
            {
                rawImageUvRectProperty.SetValue(rawImageTarget, new Rect(0f, 1f, 1f, -1f));
            }
        }

        // Renderer에 공유 FPV Texture를 적용한다.
        private void ApplyTextureToRenderer(Renderer target)
        {
            if (target == null)
            {
                return;
            }

            propertyBlock ??= new MaterialPropertyBlock();
            target.GetPropertyBlock(propertyBlock);
            propertyBlock.SetTexture("_MainTex", texture);
            propertyBlock.SetTexture("_BaseMap", texture);
            propertyBlock.SetVector("_MainTex_ST", new Vector4(1f, -1f, 0f, 1f));
            propertyBlock.SetVector("_BaseMap_ST", new Vector4(1f, -1f, 0f, 1f));
            target.SetPropertyBlock(propertyBlock);
        }

        // 공유 메모리 핸들을 닫는다.
        private void CloseSharedMemory()
        {
            accessor?.Dispose();
            mappedFile?.Dispose();
            accessor = null;
            mappedFile = null;
            lastFrameId = -1;
            LastSourceTimestampNs = 0;
            lastFrameReceivedAt = -1f;
            LastFrameAgeMilliseconds = 0d;
            nextOpenAttemptTime = 0f;
        }
    }
}
