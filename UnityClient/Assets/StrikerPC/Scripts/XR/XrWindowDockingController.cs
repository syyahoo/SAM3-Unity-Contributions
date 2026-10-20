using System;
using UnityEngine;

namespace StrikerPC.XR
{
    [DisallowMultipleComponent]
    public sealed class XrWindowDockingController : MonoBehaviour
    {
        [SerializeField] private XrHudLayout hudLayout;
        [SerializeField, Min(0f)] private float minimumPointerOffsetMagnitude = 0.15f;

        private XrDockableWindow activeWindow;
        private XrWindowSlot? sourceSlot;
        private XrWindowSlot? previewSlot;

        public XrDockableWindow ActiveWindow => activeWindow;
        public XrWindowSlot? PreviewSlot => previewSlot;

        public event Action<XrWindowSlot?> PreviewSlotChanged;
        public event Action<XrDockableWindow, XrWindowSlot, XrDockResult> DockCompleted;

        // 같은 HUD Root의 슬롯 관리자를 자동으로 연결한다.
        private void Awake()
        {
            if (hudLayout == null)
            {
                hudLayout = GetComponent<XrHudLayout>();
            }
        }

        // 생성기와 테스트에서 슬롯 관리자를 명시적으로 연결한다.
        public void Configure(XrHudLayout layout)
        {
            hudLayout = layout;
        }

        // 중앙 또는 Anchor 자체를 창으로 쓰는 레거시 구성은 이동 대상으로 허용하지 않는다.
        public bool CanBeginDock(XrDockableWindow window)
        {
            return hudLayout != null
                && window != null
                && window.HasValidIdentity
                && window.Role != XrWindowRole.Fpv
                && hudLayout.TryGetCurrentSlot(window, out XrWindowSlot currentSlot)
                && currentSlot != XrWindowSlot.Center
                && hudLayout.GetAnchor(currentSlot) != window.MovableWindow.transform;
        }

        // FPV가 아닌 창의 도킹 상호작용을 시작한다.
        public bool BeginDock(XrDockableWindow window)
        {
            if (activeWindow != null
                || !CanBeginDock(window)
                || !hudLayout.RegisterWindow(window))
            {
                return false;
            }

            hudLayout.TryGetCurrentSlot(
                window,
                out XrWindowSlot currentSlot);

            activeWindow = window;
            sourceSlot = currentSlot;
            SetPreviewSlot(null);
            return true;
        }

        // FPV 평면 기준 포인터 방향으로 유효한 주변 슬롯 미리보기를 갱신한다.
        public bool UpdateDockPreview(Vector2 pointerOffsetFromFpvCenter)
        {
            if (activeWindow == null || hudLayout == null
                || !TryResolveSlot(
                    pointerOffsetFromFpvCenter,
                    minimumPointerOffsetMagnitude,
                    out XrWindowSlot targetSlot)
                || !hudLayout.CanDock(activeWindow, targetSlot))
            {
                SetPreviewSlot(null);
                return false;
            }

            SetPreviewSlot(targetSlot);
            return true;
        }

        // 드래그 중인 창의 월드 위치에서 가장 가까운 주변 슬롯을 미리 선택한다.
        public bool UpdateDockPreview(Vector3 draggedWorldPosition)
        {
            if (activeWindow == null || hudLayout == null
                || !hudLayout.TryGetClosestPeripheralSlot(
                    draggedWorldPosition,
                    out XrWindowSlot targetSlot)
                || !hudLayout.CanDock(activeWindow, targetSlot))
            {
                SetPreviewSlot(null);
                return false;
            }

            SetPreviewSlot(targetSlot);
            return true;
        }

        // 현재 미리보기 슬롯으로 도킹을 확정한다.
        public XrDockResult CommitDock()
        {
            if (activeWindow == null || hudLayout == null || !previewSlot.HasValue)
            {
                CancelDock();
                return XrDockResult.RejectedInvalid;
            }

            XrDockableWindow window = activeWindow;
            XrWindowSlot targetSlot = previewSlot.Value;
            XrDockResult result = hudLayout.Dock(window, targetSlot);
            if (result != XrDockResult.Docked
                && result != XrDockResult.AlreadyDocked
                && sourceSlot.HasValue)
            {
                hudLayout.SnapWindowToSlot(window, sourceSlot.Value);
            }
            activeWindow = null;
            sourceSlot = null;
            SetPreviewSlot(null);
            SaveWindowTransform(window);
            if (Application.isPlaying
                && (result == XrDockResult.Docked
                    || result == XrDockResult.AlreadyDocked))
            {
                hudLayout.SaveDockLayout();
            }
            DockCompleted?.Invoke(window, targetSlot, result);
            return result;
        }

        // 도킹을 적용하지 않고 현재 상호작용을 취소한다.
        public void CancelDock()
        {
            if (activeWindow != null && hudLayout != null && sourceSlot.HasValue)
            {
                hudLayout.SnapWindowToSlot(activeWindow, sourceSlot.Value);
                SaveWindowTransform(activeWindow);
            }
            activeWindow = null;
            sourceSlot = null;
            SetPreviewSlot(null);
        }

        // 시작 시 저장된 자유 위치를 무시하고 현재 부모 슬롯의 중심으로 복원한다.
        public bool SnapWindowToCurrentSlot(XrDockableWindow window)
        {
            if (hudLayout == null || window == null
                || !hudLayout.TryGetCurrentSlot(window, out XrWindowSlot slot)
                || !hudLayout.SnapWindowToSlot(window, slot))
            {
                return false;
            }

            SaveWindowTransform(window);
            return true;
        }

        // 저장된 슬롯 배치를 우선 복원하고 저장값이 없으면 Scene 배치를 유지한다.
        public bool RestoreWindowSlot(XrDockableWindow window)
        {
            return hudLayout != null && hudLayout.RestoreWindowSlot(window);
        }

        // 창이나 HUD가 비활성화되면 남아 있는 드래그 상태를 정리한다.
        private void OnDisable()
        {
            CancelDock();
        }

        // 중앙 dead zone 밖의 지배적인 축을 상·하·좌·우 슬롯으로 변환한다.
        public static bool TryResolveSlot(
            Vector2 pointerOffsetFromFpvCenter,
            float minimumMagnitude,
            out XrWindowSlot slot)
        {
            float threshold = Mathf.Max(0f, minimumMagnitude);
            if (pointerOffsetFromFpvCenter.sqrMagnitude <= Mathf.Epsilon
                || pointerOffsetFromFpvCenter.sqrMagnitude < threshold * threshold)
            {
                slot = default;
                return false;
            }

            if (Mathf.Abs(pointerOffsetFromFpvCenter.x)
                >= Mathf.Abs(pointerOffsetFromFpvCenter.y))
            {
                slot = pointerOffsetFromFpvCenter.x >= 0f
                    ? XrWindowSlot.Right
                    : XrWindowSlot.Left;
            }
            else
            {
                slot = pointerOffsetFromFpvCenter.y >= 0f
                    ? XrWindowSlot.Upper
                    : XrWindowSlot.Lower;
            }

            return true;
        }

        // Figma 도킹 오버레이가 구독할 미리보기 상태 변경만 전달한다.
        private void SetPreviewSlot(XrWindowSlot? value)
        {
            if (previewSlot == value)
            {
                return;
            }

            previewSlot = value;
            PreviewSlotChanged?.Invoke(previewSlot);
        }

        private static void SaveWindowTransform(XrDockableWindow window)
        {
            if (Application.isPlaying && window != null && window.MovableWindow != null)
            {
                window.MovableWindow.SaveTransform();
            }
        }
    }
}
