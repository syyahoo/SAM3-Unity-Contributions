using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Transformers;

namespace StrikerPC.XR
{
    [DefaultExecutionOrder(100)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(XrMovableWindow))]
    [RequireComponent(typeof(XRGrabInteractable))]
    [RequireComponent(typeof(XRGeneralGrabTransformer))]
    public sealed class XrVerticalHudAdjuster : MonoBehaviour
    {
        [SerializeField] private XrMovableWindow movableWindow;
        [SerializeField] private XRGrabInteractable grabInteractable;
        [SerializeField] private XRGeneralGrabTransformer grabTransformer;

        private Vector3 lockedLocalPosition;
        private Quaternion lockedLocalRotation;
        private Vector3 lockedLocalScale;
        private bool poseCaptured;

        // Scene 기준 X/Z·회전·크기를 저장하고 눈높이 Y 이동만 허용한다.
        private void Awake()
        {
            ResolveReferences();
            CaptureLockedPose();
            ConfigureVerticalInteraction();
        }

        // 활성화 시 XR Grab 제한을 다시 적용한다.
        private void OnEnable()
        {
            ResolveReferences();
            CaptureLockedPose();
            ConfigureVerticalInteraction();
        }

        // 다른 상호작용이 자세를 변경해도 Y를 제외한 Scene 기준 자세를 복원한다.
        private void LateUpdate()
        {
            if (!poseCaptured)
            {
                CaptureLockedPose();
            }

            Vector3 constrainedPosition = ConstrainToVertical(
                lockedLocalPosition,
                transform.localPosition);
            if (transform.localPosition != constrainedPosition
                || transform.localRotation != lockedLocalRotation)
            {
                transform.SetLocalPositionAndRotation(
                    constrainedPosition,
                    lockedLocalRotation);
            }

            if (transform.localScale != lockedLocalScale)
            {
                transform.localScale = lockedLocalScale;
            }
        }

        // 요청 위치에서 Y만 반영하고 고정된 X/Z를 유지한다.
        public static Vector3 ConstrainToVertical(
            Vector3 lockedPosition,
            Vector3 requestedPosition)
        {
            return new Vector3(
                lockedPosition.x,
                requestedPosition.y,
                lockedPosition.z);
        }

        // 같은 GameObject의 기존 XR 이동 구성요소를 한 번 연결한다.
        private void ResolveReferences()
        {
            movableWindow ??= GetComponent<XrMovableWindow>();
            grabInteractable ??= GetComponent<XRGrabInteractable>();
            grabTransformer ??= GetComponent<XRGeneralGrabTransformer>();
        }

        // Scene에 저장된 전체 HUD 기준 자세를 잠금값으로 보존한다.
        private void CaptureLockedPose()
        {
            if (poseCaptured)
            {
                return;
            }

            lockedLocalPosition = transform.localPosition;
            lockedLocalRotation = transform.localRotation;
            lockedLocalScale = transform.localScale;
            poseCaptured = true;
        }

        // Grab 이동은 Y축만 허용하고 회전과 크기 조절을 차단한다.
        private void ConfigureVerticalInteraction()
        {
            movableWindow?.SetCenterLocked(true);
            if (grabInteractable != null)
            {
                grabInteractable.selectMode = InteractableSelectMode.Single;
                grabInteractable.throwOnDetach = false;
                grabInteractable.trackPosition = true;
                grabInteractable.trackRotation = false;
                grabInteractable.trackScale = false;
            }

            if (grabTransformer != null)
            {
                grabTransformer.allowOneHandedScaling = false;
                grabTransformer.allowTwoHandedScaling = false;
                grabTransformer.permittedDisplacementAxes =
                    XRGeneralGrabTransformer.ManipulationAxes.Y;
                grabTransformer.constrainedAxisDisplacementMode =
                    XRGeneralGrabTransformer.ConstrainedAxisDisplacementMode.ObjectRelative;
            }
        }
    }
}
