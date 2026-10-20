using UnityEngine;
using UnityEngine.InputSystem;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    public sealed class ReconWindowVisibilityToggle : MonoBehaviour
    {
        public const string DefaultBindingPath = "<XRController>{LeftHand}/secondaryButton";

        [SerializeField] private GameObject reconnaissanceWindow;
        [SerializeField] private InputActionProperty toggleAction;
        [SerializeField] private bool visibleOnEnable = true;

        private bool toggleEnabledByThis;
        private bool ownsToggleAction;

        public bool IsVisible => reconnaissanceWindow != null && reconnaissanceWindow.activeSelf;

        // 왼손 Y 입력을 준비하고 정찰 창의 초기 표시 상태를 적용한다.
        private void OnEnable()
        {
            EnsureToggleAction();
            if (toggleAction.action != null)
            {
                toggleAction.action.performed += OnTogglePerformed;
                if (!toggleAction.action.enabled)
                {
                    toggleAction.action.Enable();
                    toggleEnabledByThis = true;
                }
            }

            SetVisible(visibleOnEnable);
        }

        // 비활성화 시 등록한 입력 콜백과 소유한 활성 상태를 해제한다.
        private void OnDisable()
        {
            if (toggleAction.action == null)
            {
                return;
            }

            toggleAction.action.performed -= OnTogglePerformed;
            if (toggleEnabledByThis)
            {
                toggleAction.action.Disable();
                toggleEnabledByThis = false;
            }
        }

        // 런타임 생성한 기본 InputAction을 정리한다.
        private void OnDestroy()
        {
            if (ownsToggleAction)
            {
                toggleAction.action?.Dispose();
            }
        }

        // Prefab 인스턴스에 기존 정찰 영상 창을 연결한다.
        public void Configure(GameObject window)
        {
            reconnaissanceWindow = window;
            EnsureToggleAction();
            SetVisible(visibleOnEnable);
        }

        // 정찰 영상 창의 현재 표시 상태를 반전한다.
        public void ToggleVisibility()
        {
            SetVisible(!IsVisible);
        }

        // 정찰 영상 창 전체와 XR 상호작용을 함께 표시하거나 숨긴다.
        public void SetVisible(bool visible)
        {
            if (reconnaissanceWindow != null)
            {
                reconnaissanceWindow.SetActive(visible);
            }
        }

        // Inspector 입력이 비어 있으면 충돌하지 않는 Quest 왼손 Y를 사용한다.
        private void EnsureToggleAction()
        {
            if (toggleAction.action != null && toggleAction.action.bindings.Count > 0)
            {
                return;
            }

            toggleAction = new InputActionProperty(new InputAction(
                "Recon Window Toggle",
                InputActionType.Button,
                DefaultBindingPath));
            ownsToggleAction = true;
        }

        // 컨트롤러 입력을 정찰 창 표시 전환으로 전달한다.
        private void OnTogglePerformed(InputAction.CallbackContext _)
        {
            ToggleVisibility();
        }
    }
}
