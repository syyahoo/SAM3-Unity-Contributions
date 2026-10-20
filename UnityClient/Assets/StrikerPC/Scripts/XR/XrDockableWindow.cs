using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Transformers;

namespace StrikerPC.XR
{
    public enum XrWindowRole
    {
        Fpv = 0,
        Telemetry = 1,
        Map = 2,
        Subtitles = 3,
        Custom = 100
    }

    [DisallowMultipleComponent]
    [RequireComponent(typeof(XrMovableWindow))]
    public sealed class XrDockableWindow : MonoBehaviour
    {
        [SerializeField] private string windowId;
        [SerializeField] private XrWindowRole role = XrWindowRole.Custom;
        [SerializeField] private XrMovableWindow movableWindow;
        [SerializeField] private Transform viewRoot;
        [SerializeField] private XrWindowDockingController dockingController;

        private XRGrabInteractable grabInteractable;
        private bool dockingDragActive;
        private bool interactionSubscribed;
        private bool centerPoseCaptured;
        private Vector3 centerLocalPosition;
        private Quaternion centerLocalRotation;

        public string WindowId => windowId;
        public XrWindowRole Role => role;
        public XrMovableWindow MovableWindow => ResolveMovableWindow();
        public Transform ViewRoot => viewRoot != null ? viewRoot : transform;
        public bool HasValidIdentity => !string.IsNullOrWhiteSpace(windowId)
            && ResolveMovableWindow() != null;

        // Trigger 기반 Grab 이벤트와 슬롯 관리자를 연결한다.
        private void Awake()
        {
            ResolveDockingController();
            ConfigureSlotInteraction();
        }

        private void OnEnable()
        {
            ResolveDockingController();
            ConfigureSlotInteraction();
            SubscribeInteraction();
        }

        // Play 진입 시 저장값이 있는 주변 창만 FPV 기준 슬롯으로 복원한다.
        private void Start()
        {
            ResolveDockingController();
            ConfigureSlotInteraction();
            if (role != XrWindowRole.Fpv)
            {
                dockingController?.RestoreWindowSlot(this);
            }
        }

        // Trigger로 선택된 주변 창을 드래그하는 동안 가장 가까운 슬롯을 갱신한다.
        private void Update()
        {
            if (dockingDragActive && dockingController != null)
            {
                dockingController.UpdateDockPreview(ResolveMovableWindow().transform.position);
            }
        }

        // 중앙 FPV가 Grab 처리나 머리 회전 뒤에도 Scene의 Center 기준 자세를 유지한다.
        private void LateUpdate()
        {
            RestoreCenterPose();
        }

        // Figma ViewRoot와 런타임 창 역할을 이름에 의존하지 않고 연결한다.
        public void Configure(
            string id,
            XrWindowRole windowRole,
            XrMovableWindow targetWindow,
            Transform targetViewRoot)
        {
            windowId = string.IsNullOrWhiteSpace(id) ? string.Empty : id.Trim();
            role = windowRole;
            movableWindow = targetWindow != null ? targetWindow : GetComponent<XrMovableWindow>();
            viewRoot = targetViewRoot != null ? targetViewRoot : transform;
            ResolveDockingController();
            ConfigureSlotInteraction();
            SubscribeInteraction();
        }

        // Scene 생성기나 런타임 bootstrap이 명시적으로 슬롯 관리자를 연결한다.
        public void ConfigureDocking(XrWindowDockingController controller)
        {
            dockingController = controller;
            ConfigureSlotInteraction();
            SubscribeInteraction();
        }

        // 같은 GameObject의 공통 창 조작 컴포넌트를 지연 연결한다.
        private XrMovableWindow ResolveMovableWindow()
        {
            if (movableWindow == null)
            {
                movableWindow = GetComponent<XrMovableWindow>();
            }

            return movableWindow;
        }

        // 중앙 FPV는 세로 이동만 허용하고 주변 창은 위치·회전을 추적하되 Grab 스케일은 막는다.
        private void ConfigureSlotInteraction()
        {
            XrMovableWindow targetWindow = ResolveMovableWindow();
            bool centerLocked = role == XrWindowRole.Fpv;
            if (targetWindow != null)
            {
                targetWindow.SetCenterLocked(centerLocked);
            }
            if (centerLocked)
            {
                CaptureCenterPose();
                RestoreCenterPose();
            }
            else
            {
                centerPoseCaptured = false;
            }

            grabInteractable = targetWindow != null
                ? targetWindow.GetComponent<XRGrabInteractable>()
                : null;
            if (grabInteractable == null)
            {
                return;
            }

            bool movementEnabled = dockingController != null
                && dockingController.CanBeginDock(this);
            grabInteractable.selectMode = InteractableSelectMode.Single;
            grabInteractable.trackPosition = movementEnabled || centerLocked;
            grabInteractable.trackRotation = movementEnabled;
            grabInteractable.trackScale = false;

            XRGeneralGrabTransformer transformer =
                targetWindow.GetComponent<XRGeneralGrabTransformer>();
            if (transformer != null)
            {
                transformer.allowOneHandedScaling = false;
                transformer.allowTwoHandedScaling = false;
                if (centerLocked)
                {
                    transformer.permittedDisplacementAxes =
                        XRGeneralGrabTransformer.ManipulationAxes.Y;
                    transformer.constrainedAxisDisplacementMode =
                        XRGeneralGrabTransformer.ConstrainedAxisDisplacementMode.ObjectRelative;
                }
            }
        }

        // FPV가 중앙 역할로 구성된 시점의 Scene 로컬 자세를 기준으로 저장한다.
        private void CaptureCenterPose()
        {
            if (centerPoseCaptured || role != XrWindowRole.Fpv)
            {
                return;
            }

            XrMovableWindow targetWindow = ResolveMovableWindow();
            Transform target = targetWindow != null ? targetWindow.transform : null;
            if (target == null)
            {
                return;
            }

            centerLocalPosition = target.localPosition;
            centerLocalRotation = target.localRotation;
            centerPoseCaptured = true;
        }

        // FPV의 현재 세로 위치는 유지하고 Center 기준 X/Z와 회전을 복원한다.
        private void RestoreCenterPose()
        {
            CaptureCenterPose();
            if (!centerPoseCaptured || role != XrWindowRole.Fpv)
            {
                return;
            }

            XrMovableWindow targetWindow = ResolveMovableWindow();
            Transform target = targetWindow != null ? targetWindow.transform : null;
            Vector3 constrainedPosition = centerLocalPosition;
            if (target != null)
            {
                constrainedPosition.y = target.localPosition.y;
            }
            if (target != null
                && (target.localPosition != constrainedPosition
                    || target.localRotation != centerLocalRotation))
            {
                target.SetLocalPositionAndRotation(constrainedPosition, centerLocalRotation);
            }
        }

        private void ResolveDockingController()
        {
            if (dockingController == null)
            {
                dockingController = GetComponentInParent<XrWindowDockingController>();
            }
            if (dockingController == null)
            {
                dockingController = GetComponentInParent<XrHudLayout>()
                    ?.GetComponent<XrWindowDockingController>();
            }
        }

        private void SubscribeInteraction()
        {
            if (interactionSubscribed || grabInteractable == null)
            {
                return;
            }

            grabInteractable.selectEntered.AddListener(OnGrabSelected);
            grabInteractable.selectExited.AddListener(OnGrabReleased);
            interactionSubscribed = true;
        }

        private void UnsubscribeInteraction()
        {
            if (!interactionSubscribed || grabInteractable == null)
            {
                return;
            }

            grabInteractable.selectEntered.RemoveListener(OnGrabSelected);
            grabInteractable.selectExited.RemoveListener(OnGrabReleased);
            interactionSubscribed = false;
        }

        private void OnGrabSelected(SelectEnterEventArgs args)
        {
            RestoreCenterPose();
            dockingDragActive = dockingController != null
                && dockingController.BeginDock(this);
            if (dockingDragActive)
            {
                dockingController.UpdateDockPreview(ResolveMovableWindow().transform.position);
            }
        }

        private void OnGrabReleased(SelectExitEventArgs args)
        {
            RestoreCenterPose();
            if (!dockingDragActive || dockingController == null)
            {
                return;
            }

            dockingController.UpdateDockPreview(ResolveMovableWindow().transform.position);
            dockingController.CommitDock();
            dockingDragActive = false;
        }

        private void OnDisable()
        {
            if (dockingDragActive)
            {
                dockingController?.CancelDock();
                dockingDragActive = false;
            }

            UnsubscribeInteraction();
        }

        private void OnDestroy()
        {
            UnsubscribeInteraction();
        }
    }
}
