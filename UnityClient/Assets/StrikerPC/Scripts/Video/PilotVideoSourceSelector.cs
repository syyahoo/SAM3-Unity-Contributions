using UnityEngine;

namespace StrikerPC.Video
{
    public enum PilotVideoSourceMode
    {
        SharedMemory = 0,
        LiveKit = 1
    }

    public sealed class PilotVideoSourceSelector : MonoBehaviour
    {
        [SerializeField] private PilotVideoSourceMode sourceMode = PilotVideoSourceMode.SharedMemory;
        [SerializeField] private PilotSharedFrameTexture sharedMemorySource;
        [SerializeField] private DroneAnchorOverlay localAnchorOverlay;
        [SerializeField] private Renderer[] targetRenderers;
        [SerializeField] private Texture placeholderTexture;

        private MaterialPropertyBlock propertyBlock;
        private Texture liveKitTexture;

        public PilotVideoSourceMode SourceMode => sourceMode;
        public bool UsesLiveKit => sourceMode == PilotVideoSourceMode.LiveKit;

        // 저장된 영상 소스 모드를 화면에 적용한다.
        private void Awake()
        {
            ApplySourceMode();
        }

        // Inspector 변경 내용을 즉시 반영한다.
        private void OnValidate()
        {
            if (Application.isPlaying)
            {
                ApplySourceMode();
            }
        }

        // 사용할 영상 소스를 지정한다.
        public void SetSourceMode(PilotVideoSourceMode value)
        {
            if (sourceMode == value)
            {
                ApplySourceMode();
                return;
            }

            sourceMode = value;
            ApplySourceMode();
        }

        // LiveKit에서 수신한 텍스처를 기존 FPV 화면에 연결한다.
        public void SetLiveKitTexture(Texture texture)
        {
            liveKitTexture = texture;
            if (UsesLiveKit)
            {
                ApplyTexture(texture, false);
            }
        }

        // 연결 종료 시 원격 영상 텍스처를 제거한다.
        public void ClearLiveKitTexture()
        {
            liveKitTexture = null;
            if (UsesLiveKit)
            {
                ApplyTexture(placeholderTexture, false);
            }
        }

        // 소스에 맞춰 공유 메모리 수신과 화면 텍스처를 전환한다.
        private void ApplySourceMode()
        {
            bool useSharedMemory = sourceMode == PilotVideoSourceMode.SharedMemory;
            if (sharedMemorySource != null)
            {
                sharedMemorySource.enabled = useSharedMemory;
            }

            if (localAnchorOverlay != null)
            {
                localAnchorOverlay.enabled = true;
            }

            Texture selectedTexture = useSharedMemory
                ? sharedMemorySource?.Texture
                : liveKitTexture;
            ApplyTexture(
                selectedTexture != null ? selectedTexture : placeholderTexture,
                useSharedMemory);
        }

        // 모든 기존 FPV Renderer에 같은 텍스처를 적용한다.
        private void ApplyTexture(Texture texture, bool flipVertical)
        {
            Texture selectedTexture = texture != null ? texture : Texture2D.blackTexture;
            Vector4 textureTransform = flipVertical
                ? new Vector4(1f, -1f, 0f, 1f)
                : new Vector4(1f, 1f, 0f, 0f);
            propertyBlock ??= new MaterialPropertyBlock();
            if (targetRenderers == null)
            {
                return;
            }

            foreach (Renderer targetRenderer in targetRenderers)
            {
                if (targetRenderer == null)
                {
                    continue;
                }

                targetRenderer.GetPropertyBlock(propertyBlock);
                propertyBlock.SetTexture("_MainTex", selectedTexture);
                propertyBlock.SetTexture("_BaseMap", selectedTexture);
                propertyBlock.SetVector("_MainTex_ST", textureTransform);
                propertyBlock.SetVector("_BaseMap_ST", textureTransform);
                targetRenderer.SetPropertyBlock(propertyBlock);
            }
        }
    }
}
