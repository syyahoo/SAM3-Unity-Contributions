using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Button))]
    [RequireComponent(typeof(BoxCollider))]
    [RequireComponent(typeof(XRSimpleInteractable))]
    public sealed class XrUiButtonPressRelay : MonoBehaviour
    {
        [SerializeField] private Button button;
        [SerializeField] private BoxCollider hitCollider;
        [SerializeField, Min(1f)] private float hitDepth = 20f;

        private XRSimpleInteractable interactable;

        // XR Trigger 선택을 uGUI 버튼 클릭으로 전달한다.
        private void Awake()
        {
            ResolveReferences();
            ConfigureInteraction();
            interactable.selectEntered.AddListener(OnSelectEntered);
        }

        // Scene 생성 코드에서 버튼 참조와 입력 영역을 직렬화한다.
        public void Configure(Button targetButton)
        {
            button = targetButton;
            ResolveReferences();
            ConfigureInteraction();
        }

        // 테스트와 XR 선택이 동일한 버튼 클릭 경로를 사용한다.
        public void Press()
        {
            if (button != null && button.IsInteractable())
            {
                button.onClick.Invoke();
            }
        }

        private void OnSelectEntered(SelectEnterEventArgs args)
        {
            Press();
        }

        // 버튼 Rect와 XR Collider 크기를 동일하게 유지한다.
        private void ConfigureInteraction()
        {
            RectTransform rect = transform as RectTransform;
            if (rect == null || hitCollider == null || interactable == null)
            {
                return;
            }

            Vector2 size = rect.rect.size;
            hitCollider.center = Vector3.zero;
            hitCollider.size = new Vector3(
                Mathf.Max(1f, size.x),
                Mathf.Max(1f, size.y),
                hitDepth);
            hitCollider.isTrigger = true;
            interactable.colliders.Clear();
            interactable.colliders.Add(hitCollider);
        }

        private void ResolveReferences()
        {
            button ??= GetComponent<Button>();
            hitCollider ??= GetComponent<BoxCollider>();
            interactable ??= GetComponent<XRSimpleInteractable>();
        }

        private void OnDestroy()
        {
            if (interactable != null)
            {
                interactable.selectEntered.RemoveListener(OnSelectEntered);
            }
        }
    }
}
