using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(BoxCollider))]
    [RequireComponent(typeof(XRSimpleInteractable))]
    public sealed class XrWindowResizeHandle : MonoBehaviour
    {
        [SerializeField] private XrMovableWindow window;
        [SerializeField] private Vector2 corner = Vector2.one;
        [SerializeField] private Graphic highlightGraphic;
        [SerializeField, Min(4f)] private float edgeHitThickness = 14f;
        [SerializeField, Min(4f)] private float cornerHitSize = 18f;
        [SerializeField, Min(0f)] private float edgeInset = 12f;
        [SerializeField, Min(1f)] private float hitDepth = 20f;
        [SerializeField] private Color idleColor = new Color(0.2f, 0.75f, 1f, 0f);
        [SerializeField] private Color hoverColor = new Color(0.2f, 0.75f, 1f, 0f);
        [SerializeField] private Color selectedColor = new Color(0.35f, 0.9f, 1f, 0f);

        private XRSimpleInteractable interactable;
        private IXRSelectInteractor selectingInteractor;
        private RectTransform handleRect;
        private BoxCollider hitCollider;
        private Vector2 lastColliderSize = new Vector2(-1f, -1f);

        // 모서리의 XR 선택 이벤트와 하이라이트를 준비한다.
        private void Awake()
        {
            interactable = GetComponent<XRSimpleInteractable>();
            handleRect = transform as RectTransform;
            hitCollider = GetComponent<BoxCollider>();
            if (!IsCornerDirection(corner))
            {
                interactable.enabled = false;
                hitCollider.enabled = false;
                SetHighlight(idleColor);
                return;
            }
            ApplyBorderHitGeometry();
            interactable.hoverEntered.AddListener(OnHoverEntered);
            interactable.hoverExited.AddListener(OnHoverExited);
            interactable.selectEntered.AddListener(OnSelectEntered);
            interactable.selectExited.AddListener(OnSelectExited);
            SetHighlight(idleColor);
        }

        // 선택 중인 한 손 위치를 창 크기에 반영한다.
        private void Update()
        {
            SynchronizeColliderSize();
            if (selectingInteractor == null || window == null)
            {
                return;
            }

            Transform attachTransform = selectingInteractor.GetAttachTransform(interactable);
            if (attachTransform != null)
            {
                window.ResizeFromHandle(corner, attachTransform.position);
            }
        }

        // 고정 또는 최소화 상태에 맞춰 모서리 입력을 전환한다.
        public void SetInteractionEnabled(bool value)
        {
            if (interactable == null)
            {
                interactable = GetComponent<XRSimpleInteractable>();
            }

            interactable.enabled = value && IsCornerDirection(corner);
            if (hitCollider != null)
            {
                hitCollider.enabled = interactable.enabled;
            }
            if (!value)
            {
                if (selectingInteractor != null)
                {
                    window?.EndResize();
                }

                selectingInteractor = null;
                SetHighlight(idleColor);
            }
        }

        // 포인터가 모서리에 닿으면 하이라이트한다.
        private void OnHoverEntered(HoverEnterEventArgs args)
        {
            SetHighlight(hoverColor);
        }

        // 포인터가 모서리에서 벗어나면 하이라이트를 제거한다.
        private void OnHoverExited(HoverExitEventArgs args)
        {
            if (selectingInteractor == null)
            {
                SetHighlight(idleColor);
            }
        }

        // 모서리 선택 시 반대쪽 모서리를 기준으로 리사이즈를 시작한다.
        private void OnSelectEntered(SelectEnterEventArgs args)
        {
            if (window == null || !window.BeginResize(corner))
            {
                return;
            }

            selectingInteractor = args.interactorObject;
            SetHighlight(selectedColor);
        }

        // 모서리 선택 해제 시 창 크기를 저장한다.
        private void OnSelectExited(SelectExitEventArgs args)
        {
            if (selectingInteractor != args.interactorObject)
            {
                return;
            }

            selectingInteractor = null;
            window?.EndResize();
            SetHighlight(idleColor);
        }

        // 모서리 시각 요소의 색상을 변경한다.
        private void SetHighlight(Color color)
        {
            if (highlightGraphic != null)
            {
                color.a = 0f;
                highlightGraphic.color = color;
            }
        }

        // 한 축이 0이면 얇은 변, 두 축이 모두 있으면 작은 모서리 입력 영역을 구성한다.
        private void ApplyBorderHitGeometry()
        {
            if (handleRect == null)
            {
                return;
            }

            Vector2 direction = NormalizeResizeDirection(corner);
            if (direction == Vector2.zero)
            {
                direction = Vector2.one;
                corner = direction;
            }

            handleRect.pivot = new Vector2(0.5f, 0.5f);
            if (direction.x == 0f)
            {
                float verticalAnchor = direction.y > 0f ? 1f : 0f;
                handleRect.anchorMin = new Vector2(0f, verticalAnchor);
                handleRect.anchorMax = new Vector2(1f, verticalAnchor);
                handleRect.anchoredPosition = Vector2.zero;
                handleRect.sizeDelta = new Vector2(-edgeInset * 2f, edgeHitThickness);
            }
            else if (direction.y == 0f)
            {
                float horizontalAnchor = direction.x > 0f ? 1f : 0f;
                handleRect.anchorMin = new Vector2(horizontalAnchor, 0f);
                handleRect.anchorMax = new Vector2(horizontalAnchor, 1f);
                handleRect.anchoredPosition = Vector2.zero;
                handleRect.sizeDelta = new Vector2(edgeHitThickness, -edgeInset * 2f);
            }
            else
            {
                handleRect.anchorMin = handleRect.anchorMax = new Vector2(
                    direction.x > 0f ? 1f : 0f,
                    direction.y > 0f ? 1f : 0f);
                handleRect.anchoredPosition = Vector2.zero;
                handleRect.sizeDelta = Vector2.one * cornerHitSize;
            }

            SetHighlight(idleColor);
            SynchronizeColliderSize();
        }

        // RectTransform의 얇은 테두리 영역과 XR Collider 크기를 동일하게 유지한다.
        private void SynchronizeColliderSize()
        {
            if (handleRect == null || hitCollider == null)
            {
                return;
            }

            Vector2 size = handleRect.rect.size;
            if (size == lastColliderSize)
            {
                return;
            }

            lastColliderSize = size;
            hitCollider.center = Vector3.zero;
            hitCollider.size = new Vector3(
                Mathf.Max(1f, size.x),
                Mathf.Max(1f, size.y),
                hitDepth);
        }

        // 0인 축은 변 리사이즈로 유지하고 나머지는 방향 부호만 사용한다.
        private static Vector2 NormalizeResizeDirection(Vector2 direction)
        {
            return new Vector2(
                Mathf.Abs(direction.x) <= 0.0001f ? 0f : Mathf.Sign(direction.x),
                Mathf.Abs(direction.y) <= 0.0001f ? 0f : Mathf.Sign(direction.y));
        }

        // 두 축이 모두 지정된 모서리만 리사이즈 입력으로 허용한다.
        private static bool IsCornerDirection(Vector2 direction)
        {
            Vector2 normalized = NormalizeResizeDirection(direction);
            return normalized.x != 0f && normalized.y != 0f;
        }

        // 등록한 XR 이벤트를 해제한다.
        private void OnDestroy()
        {
            if (interactable == null)
            {
                return;
            }

            interactable.hoverEntered.RemoveListener(OnHoverEntered);
            interactable.hoverExited.RemoveListener(OnHoverExited);
            interactable.selectEntered.RemoveListener(OnSelectEntered);
            interactable.selectExited.RemoveListener(OnSelectExited);
        }
    }
}
