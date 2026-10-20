using System;
using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.Video
{
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public sealed class LiveKitVideoSurface : MonoBehaviour
    {
        [Header("LiveKit Track")]
        [SerializeField] private string publisherIdentity;
        [SerializeField] private string trackName;

        [Header("Output")]
        [SerializeField] private Renderer targetRenderer;
        [SerializeField] private Text statusLabel;
        [SerializeField] private Texture placeholderTexture;

        private MaterialPropertyBlock propertyBlock;

        public string PublisherIdentity => publisherIdentity;
        public string TrackName => trackName;

        // 활성화 시 실제 영상이 오기 전까지 대기 화면을 표시한다.
        private void OnEnable()
        {
            ClearTexture();
        }

        // 설정한 Publisher와 Track 이름이 수신 영상과 일치하는지 확인한다.
        public bool Matches(string participantIdentity, string publishedTrackName)
        {
            return MatchesFilter(publisherIdentity, participantIdentity) &&
                   MatchesFilter(trackName, publishedTrackName);
        }

        // LiveKit에서 수신한 텍스처를 정찰 드론 화면에 표시한다.
        public void SetTexture(Texture texture)
        {
            ApplyTexture(texture != null ? texture : placeholderTexture);
            if (statusLabel != null)
            {
                statusLabel.text = texture != null ? "LIVEKIT 스트리밍" : "LIVEKIT 대기 중";
            }
        }

        // 연결 종료 시 화면과 상태 문구를 대기 상태로 되돌린다.
        public void ClearTexture()
        {
            ApplyTexture(placeholderTexture);
            if (statusLabel != null)
            {
                statusLabel.text = "LIVEKIT 대기 중";
            }
        }

        // 필터가 비어 있으면 수신을 보류하고 설정된 값은 정확히 비교한다.
        private static bool MatchesFilter(string filter, string value)
        {
            return !string.IsNullOrWhiteSpace(filter) &&
                   string.Equals(filter, value, StringComparison.Ordinal);
        }

        // Renderer 재질 인스턴스를 만들지 않고 텍스처만 갱신한다.
        private void ApplyTexture(Texture texture)
        {
            if (targetRenderer == null)
            {
                return;
            }

            propertyBlock ??= new MaterialPropertyBlock();
            targetRenderer.GetPropertyBlock(propertyBlock);
            Texture selectedTexture = texture != null ? texture : Texture2D.blackTexture;
            propertyBlock.SetTexture("_MainTex", selectedTexture);
            propertyBlock.SetTexture("_BaseMap", selectedTexture);
            propertyBlock.SetVector("_MainTex_ST", new Vector4(1f, 1f, 0f, 0f));
            propertyBlock.SetVector("_BaseMap_ST", new Vector4(1f, 1f, 0f, 0f));
            targetRenderer.SetPropertyBlock(propertyBlock);
        }
    }
}
