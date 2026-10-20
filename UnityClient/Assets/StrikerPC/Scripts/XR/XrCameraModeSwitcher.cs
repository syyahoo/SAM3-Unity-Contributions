using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

namespace StrikerPC.XR
{
    public sealed class XrCameraModeSwitcher : MonoBehaviour
    {
        [SerializeField] private Camera desktopCamera;
        [SerializeField] private Camera xrCamera;
        [SerializeField] private AudioListener xrAudioListener;
        [Tooltip("XR 모드에서만 활성화할 Meta Passthrough 관리자입니다.")]
        [SerializeField] private Behaviour passthroughManager;
        [SerializeField] private GameObject xrHudRoot;
        [Tooltip("HMD 없이 XR HUD 배치를 확인할 때만 사용합니다.")]
        [SerializeField] private bool forceXrPreview;

        private static readonly List<XRDisplaySubsystem> DisplaySubsystems = new();
        private bool isXrMode;

        public bool IsXrMode => isXrMode;
        public bool IsPassthroughEnabled => passthroughManager != null && passthroughManager.enabled;

        // 포커스 모드가 기존 상태를 보존하며 Passthrough를 전환할 수 있게 한다.
        public void SetPassthroughEnabled(bool enabled)
        {
            if (passthroughManager != null)
            {
                passthroughManager.enabled = enabled;
            }
        }

        // Meta Composition Layer가 시작되기 전에 Passthrough 관리자를 활성화한다.
        private void Awake()
        {
            SetPassthroughEnabled(true);
        }

        // XR Loader가 준비될 때까지 잠시 확인한 뒤 사용할 카메라를 선택한다.
        private IEnumerator Start()
        {
            ApplyViewMode(false);
            WaitForSecondsRealtime retryDelay = new WaitForSecondsRealtime(0.1f);
            for (int attempt = 0; attempt < 30; attempt++)
            {
                if (forceXrPreview || IsXrDisplayRunning())
                {
                    ApplyViewMode(true);
                    yield break;
                }

                yield return retryDelay;
            }
        }

        // 실행 중인 XR Display Subsystem이 있는지 확인한다.
        private static bool IsXrDisplayRunning()
        {
            DisplaySubsystems.Clear();
            SubsystemManager.GetSubsystems(DisplaySubsystems);
            foreach (XRDisplaySubsystem subsystem in DisplaySubsystems)
            {
                if (subsystem != null && subsystem.running)
                {
                    return true;
                }
            }

            return false;
        }

        // 데스크톱과 XR 카메라를 중복 렌더링하지 않도록 전환한다.
        private void ApplyViewMode(bool useXr)
        {
            isXrMode = useXr;
            if (desktopCamera != null)
            {
                desktopCamera.enabled = !useXr;
                desktopCamera.gameObject.tag = useXr ? "Untagged" : "MainCamera";
            }

            if (xrCamera != null)
            {
                xrCamera.enabled = useXr;
                xrCamera.gameObject.tag = useXr ? "MainCamera" : "Untagged";
            }

            if (xrAudioListener != null)
            {
                xrAudioListener.enabled = useXr;
            }

            if (xrHudRoot != null)
            {
                xrHudRoot.SetActive(useXr);
            }
        }
    }
}
