using System;
using System.Collections.Generic;
using StrikerPC.State;
using UnityEngine;

namespace StrikerPC.XR
{
    public enum XrWindowSlot
    {
        Center,
        Left,
        Right,
        Upper,
        Lower
    }

    public enum XrDockResult
    {
        Docked = 0,
        AlreadyDocked = 1,
        RejectedReserved = 2,
        RejectedOccupied = 3,
        RejectedInvalid = 4
    }

    public sealed class XrHudLayout : MonoBehaviour
    {
        private const string DockPersistencePrefix = "StrikerPC.XrHudLayout";

        private static readonly XrWindowSlot[] Slots =
        {
            XrWindowSlot.Center,
            XrWindowSlot.Left,
            XrWindowSlot.Right,
            XrWindowSlot.Upper,
            XrWindowSlot.Lower
        };

        [SerializeField] private Transform centerAnchor;
        [SerializeField] private Transform leftAnchor;
        [SerializeField] private Transform rightAnchor;
        [SerializeField] private Transform upperAnchor;
        [SerializeField] private Transform lowerAnchor;
        [SerializeField] private XrDockableWindow[] dockableWindows;
        [SerializeField] private bool createTelemetryHud = true;
        [SerializeField] private TelloTelemetryHud telemetryHud;

        private readonly List<XrDockableWindow> registeredWindows =
            new List<XrDockableWindow>();
        private readonly Dictionary<XrWindowSlot, Transform> legacyOccupants =
            new Dictionary<XrWindowSlot, Transform>();
        private bool registryInitialized;

        public event Action<XrDockableWindow, XrWindowSlot> DockChanged;

        // Scene에 배치된 Tello 텔레메트리 창의 표시 상태를 적용한다.
        private void Awake()
        {
            RefreshDockableWindows();
            ParentPeripheralAnchorsToFpv();
            XrHudVisibilityStatus.EnsureCreated(this);
            DepthDistanceHud.EnsureCreated(this);
            if (telemetryHud != null)
            {
                telemetryHud.gameObject.SetActive(createTelemetryHud);
            }
        }

        // 지정한 XR 창 슬롯의 Anchor를 반환한다.
        public Transform GetAnchor(XrWindowSlot slot)
        {
            return slot switch
            {
                XrWindowSlot.Center => centerAnchor,
                XrWindowSlot.Left => leftAnchor,
                XrWindowSlot.Right => rightAnchor,
                XrWindowSlot.Upper => upperAnchor,
                XrWindowSlot.Lower => lowerAnchor,
                _ => null
            };
        }

        // 지정한 슬롯의 Anchor가 준비되었는지 확인한다.
        public bool TryGetAnchor(XrWindowSlot slot, out Transform anchor)
        {
            anchor = GetAnchor(slot);
            return anchor != null;
        }

        // 주변 슬롯 Anchor를 FPV 아래로 묶어 FPV의 세로 이동을 모든 창에 전달한다.
        private bool ParentPeripheralAnchorsToFpv()
        {
            if (!TryGetWindow(XrWindowRole.Fpv, out XrDockableWindow fpv)
                || fpv.MovableWindow == null)
            {
                return false;
            }

            Transform fpvRoot = fpv.MovableWindow.transform;
            bool foundPeripheralAnchor = false;
            foreach (XrWindowSlot slot in Slots)
            {
                if (slot == XrWindowSlot.Center)
                {
                    continue;
                }

                Transform anchor = GetAnchor(slot);
                if (anchor == null)
                {
                    continue;
                }

                foundPeripheralAnchor = true;
                if (anchor == fpvRoot || fpvRoot.IsChildOf(anchor))
                {
                    return false;
                }
                if (anchor.parent != fpvRoot)
                {
                    anchor.SetParent(fpvRoot, true);
                }
            }

            return foundPeripheralAnchor;
        }

        // 월드 위치에서 가장 가까운 주변 슬롯을 찾고 중앙 FPV 슬롯은 제외한다.
        public bool TryGetClosestPeripheralSlot(
            Vector3 worldPosition,
            out XrWindowSlot closestSlot)
        {
            float closestDistance = float.PositiveInfinity;
            closestSlot = default;
            bool found = false;
            foreach (XrWindowSlot candidate in Slots)
            {
                if (candidate == XrWindowSlot.Center)
                {
                    continue;
                }

                Transform anchor = GetAnchor(candidate);
                if (anchor == null)
                {
                    continue;
                }

                float distance = (anchor.position - worldPosition).sqrMagnitude;
                if (distance >= closestDistance)
                {
                    continue;
                }

                closestDistance = distance;
                closestSlot = candidate;
                found = true;
            }

            return found;
        }

        // 역할에 해당하는 도킹 가능 창을 찾는다.
        public bool TryGetWindow(XrWindowRole role, out XrDockableWindow window)
        {
            if (role == XrWindowRole.Custom)
            {
                window = null;
                return false;
            }

            foreach (XrDockableWindow candidate in GetRegisteredWindows())
            {
                if (candidate != null && candidate.Role == role)
                {
                    window = candidate;
                    return true;
                }
            }

            window = null;
            return false;
        }

        // GameObject 이름과 독립적인 창 ID로 도킹 가능 창을 찾는다.
        public bool TryGetWindow(string windowId, out XrDockableWindow window)
        {
            if (string.IsNullOrWhiteSpace(windowId))
            {
                window = null;
                return false;
            }

            foreach (XrDockableWindow candidate in GetRegisteredWindows())
            {
                if (candidate != null && string.Equals(
                        candidate.WindowId,
                        windowId,
                        StringComparison.Ordinal))
                {
                    window = candidate;
                    return true;
                }
            }

            window = null;
            return false;
        }

        // 슬롯에 직접 배치된 창을 반환한다.
        public bool TryGetOccupant(XrWindowSlot slot, out XrDockableWindow window)
        {
            foreach (XrDockableWindow candidate in GetRegisteredWindows())
            {
                if (TryGetCurrentSlot(candidate, out XrWindowSlot currentSlot)
                    && currentSlot == slot)
                {
                    window = candidate;
                    return true;
                }
            }

            window = null;
            return false;
        }

        // Scene 직렬화 배열과 현재 자식 창으로 도킹 Registry를 다시 구성한다.
        public void RefreshDockableWindows()
        {
            registeredWindows.Clear();
            if (dockableWindows != null)
            {
                foreach (XrDockableWindow window in dockableWindows)
                {
                    AddWindowToRegistry(window);
                }
            }

            foreach (XrDockableWindow window in GetComponentsInChildren<XrDockableWindow>(true))
            {
                AddWindowToRegistry(window);
            }

            registryInitialized = true;
        }

        // 현재 Window ID별 슬롯 배치를 다음 실행을 위해 저장한다.
        public void SaveDockLayout()
        {
            foreach (XrDockableWindow window in GetRegisteredWindows())
            {
                if (window != null && window.HasValidIdentity
                    && TryGetCurrentSlot(window, out XrWindowSlot slot))
                {
                    PlayerPrefs.SetInt(GetDockPersistenceKey(window.WindowId), (int)slot);
                }
            }

            PlayerPrefs.Save();
        }

        // 저장된 슬롯을 복원하고 저장값이 없거나 유효하지 않으면 Scene 배치를 사용한다.
        public bool RestoreWindowSlot(XrDockableWindow window)
        {
            if (window == null || !window.HasValidIdentity || !RegisterWindow(window))
            {
                return false;
            }

            string key = GetDockPersistenceKey(window.WindowId);
            if (!PlayerPrefs.HasKey(key))
            {
                return true;
            }

            int storedSlot = PlayerPrefs.GetInt(key);
            if (!Enum.IsDefined(typeof(XrWindowSlot), storedSlot))
            {
                PlayerPrefs.DeleteKey(key);
                PlayerPrefs.Save();
                return true;
            }

            XrWindowSlot savedSlot = (XrWindowSlot)storedSlot;
            if ((savedSlot == XrWindowSlot.Center) != (window.Role == XrWindowRole.Fpv))
            {
                PlayerPrefs.DeleteKey(key);
                PlayerPrefs.Save();
                return true;
            }

            if (TryGetCurrentSlot(window, out XrWindowSlot currentSlot)
                && currentSlot == savedSlot)
            {
                return SnapWindowToCurrentSlot(window);
            }

            XrDockResult result = Dock(window, savedSlot);
            if (result == XrDockResult.Docked || result == XrDockResult.AlreadyDocked)
            {
                return true;
            }

            return false;
        }

        // 런타임에 생성하거나 Figma ViewRoot를 연결한 창을 Registry에 추가한다.
        public bool RegisterWindow(XrDockableWindow window)
        {
            if (!registryInitialized)
            {
                RefreshDockableWindows();
            }
            if (window == null || !window.HasValidIdentity
                || !HasCompatibleRoleAndSlot(window))
            {
                return false;
            }

            foreach (XrDockableWindow registered in registeredWindows)
            {
                if (registered == window)
                {
                    RemoveLegacyOccupant(window.MovableWindow.transform);
                    return true;
                }
                if (registered != null && string.Equals(
                        registered.WindowId,
                        window.WindowId,
                        StringComparison.Ordinal))
                {
                    return false;
                }
                if (registered != null
                    && window.Role != XrWindowRole.Custom
                    && registered.Role == window.Role)
                {
                    return false;
                }
                if (registered != null
                    && TryGetCurrentSlot(registered, out XrWindowSlot registeredSlot)
                    && TryGetCurrentSlot(window, out XrWindowSlot windowSlot)
                    && registeredSlot == windowSlot)
                {
                    return false;
                }
            }

            RemoveLegacyOccupant(window.MovableWindow.transform);
            registeredWindows.Add(window);
            return true;
        }

        // 창의 현재 직계 부모를 기준으로 슬롯을 확인한다.
        public bool TryGetCurrentSlot(XrDockableWindow window, out XrWindowSlot slot)
        {
            Transform windowTransform = window != null && window.MovableWindow != null
                ? window.MovableWindow.transform
                : null;
            if (windowTransform != null)
            {
                foreach (XrWindowSlot candidate in Slots)
                {
                    Transform anchor = GetAnchor(candidate);
                    if (anchor != null
                        && (windowTransform == anchor || windowTransform.parent == anchor))
                    {
                        slot = candidate;
                        return true;
                    }
                }
            }

            slot = default;
            return false;
        }

        // FPV Center 예약과 주변 슬롯 교환 가능 여부를 반영해 도킹을 검증한다.
        public bool CanDock(XrDockableWindow window, XrWindowSlot targetSlot)
        {
            if (window == null || !window.HasValidIdentity
                || targetSlot == XrWindowSlot.Center
                || window.Role == XrWindowRole.Fpv)
            {
                return false;
            }

            Transform windowTransform = window.MovableWindow.transform;
            Transform targetAnchor = GetAnchor(targetSlot);
            if (targetAnchor == null
                || IsConfiguredAnchor(windowTransform)
                || targetAnchor == windowTransform
                || targetAnchor.IsChildOf(windowTransform))
            {
                return false;
            }
            if (TryGetCurrentSlot(window, out XrWindowSlot currentSlot)
                && currentSlot == XrWindowSlot.Center)
            {
                return false;
            }

            if (TryGetLegacyOccupant(targetSlot, out Transform legacy)
                && legacy != windowTransform)
            {
                return false;
            }

            if (!TryGetOccupant(targetSlot, out XrDockableWindow occupant)
                || occupant == window)
            {
                return true;
            }

            return occupant.HasValidIdentity
                && occupant.Role != XrWindowRole.Fpv
                && !IsConfiguredAnchor(occupant.MovableWindow.transform)
                && TryGetCurrentSlot(window, out currentSlot)
                && currentSlot != XrWindowSlot.Center;
        }

        // 비어 있는 주변 슬롯으로 창을 이동한다.
        public XrDockResult Dock(XrDockableWindow window, XrWindowSlot targetSlot)
        {
            if (window == null || !window.HasValidIdentity)
            {
                return XrDockResult.RejectedInvalid;
            }
            if (targetSlot == XrWindowSlot.Center || window.Role == XrWindowRole.Fpv)
            {
                return XrDockResult.RejectedReserved;
            }
            if (!RegisterWindow(window))
            {
                return XrDockResult.RejectedInvalid;
            }
            if (TryGetCurrentSlot(window, out XrWindowSlot sourceSlot)
                && sourceSlot == targetSlot)
            {
                if (IsSlotOccupied(targetSlot, window, null))
                {
                    return XrDockResult.RejectedOccupied;
                }
                return MoveWindow(window, targetSlot)
                    ? XrDockResult.AlreadyDocked
                    : XrDockResult.RejectedInvalid;
            }
            if (!CanDock(window, targetSlot))
            {
                return IsSlotOccupied(targetSlot, window, null)
                    ? XrDockResult.RejectedOccupied
                    : XrDockResult.RejectedInvalid;
            }

            if (TryGetOccupant(targetSlot, out XrDockableWindow occupant)
                && occupant != window)
            {
                if (!TryGetCurrentSlot(window, out XrWindowSlot swapSourceSlot)
                    || swapSourceSlot == XrWindowSlot.Center
                    || !MoveWindow(occupant, swapSourceSlot)
                    || !MoveWindow(window, targetSlot))
                {
                    return XrDockResult.RejectedInvalid;
                }

                DockChanged?.Invoke(occupant, swapSourceSlot);
                DockChanged?.Invoke(window, targetSlot);
                return XrDockResult.Docked;
            }

            if (!MoveWindow(window, targetSlot))
            {
                return XrDockResult.RejectedInvalid;
            }

            DockChanged?.Invoke(window, targetSlot);
            return XrDockResult.Docked;
        }

        // 취소하거나 무효한 위치에 놓은 창을 원래 슬롯 중심으로 되돌린다.
        public bool SnapWindowToSlot(XrDockableWindow window, XrWindowSlot slot)
        {
            if (window == null || !window.HasValidIdentity
                || !RegisterWindow(window)
                || (slot == XrWindowSlot.Center) != (window.Role == XrWindowRole.Fpv)
                || IsSlotOccupied(slot, window, null))
            {
                return false;
            }

            return MoveWindow(window, slot);
        }

        // 창 오브젝트를 지정한 XR 슬롯에 배치한다.
        public bool AttachWindow(GameObject window, XrWindowSlot slot, bool resetLocalTransform = true)
        {
            Transform anchor = GetAnchor(slot);
            if (window == null || anchor == null || slot == XrWindowSlot.Center)
            {
                return false;
            }

            XrDockableWindow dockable = window.GetComponent<XrDockableWindow>();
            if (dockable != null)
            {
                return resetLocalTransform
                    ? AttachDockableWindowSnapped(dockable, slot)
                    : AttachDockableWindowPreservingLocalTransform(dockable, slot);
            }

            Transform windowTransform = window.transform;
            if (IsConfiguredAnchor(windowTransform)
                || anchor == windowTransform
                || anchor.IsChildOf(windowTransform)
                || IsSlotOccupied(slot, null, windowTransform))
            {
                return false;
            }

            RemoveLegacyOccupant(windowTransform);
            windowTransform.SetParent(anchor, false);
            if (resetLocalTransform)
            {
                windowTransform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
            }
            legacyOccupants[slot] = windowTransform;

            return true;
        }

        // 지정한 XR 창 슬롯의 표시 상태를 변경한다.
        public void SetWindowVisible(XrWindowSlot slot, bool visible)
        {
            Transform anchor = GetAnchor(slot);
            if (anchor != null)
            {
                anchor.gameObject.SetActive(visible);
            }
        }

        // 도킹 가능 창을 슬롯 중심에 배치하고 도킹 결과를 bool로 변환한다.
        private bool AttachDockableWindowSnapped(XrDockableWindow window, XrWindowSlot slot)
        {
            XrDockResult result = Dock(window, slot);
            return result == XrDockResult.Docked || result == XrDockResult.AlreadyDocked;
        }

        // 기존 AttachWindow 계약에 따라 창의 로컬 Transform을 유지해 부모만 변경한다.
        private bool AttachDockableWindowPreservingLocalTransform(
            XrDockableWindow window,
            XrWindowSlot slot)
        {
            if (window == null || !window.HasValidIdentity
                || slot == XrWindowSlot.Center || window.Role == XrWindowRole.Fpv
                || !RegisterWindow(window))
            {
                return false;
            }
            if (TryGetCurrentSlot(window, out XrWindowSlot sourceSlot)
                && sourceSlot == slot)
            {
                return !IsSlotOccupied(slot, window, null);
            }
            if (!CanDock(window, slot))
            {
                return false;
            }

            Transform anchor = GetAnchor(slot);
            Transform windowTransform = window.MovableWindow.transform;
            windowTransform.SetParent(anchor, false);
            DockChanged?.Invoke(window, slot);
            return true;
        }

        // 창의 고유 스케일을 유지하면서 슬롯 중심에 배치한다.
        private bool MoveWindow(XrDockableWindow window, XrWindowSlot slot)
        {
            Transform anchor = GetAnchor(slot);
            Transform windowTransform = window != null && window.MovableWindow != null
                ? window.MovableWindow.transform
                : null;
            if (anchor == null || windowTransform == null
                || anchor == windowTransform || anchor.IsChildOf(windowTransform))
            {
                return false;
            }

            windowTransform.SetParent(anchor, true);
            windowTransform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
            return true;
        }

        // 현재 부모 슬롯을 유지하면서 창을 슬롯 중심으로 정렬한다.
        private bool SnapWindowToCurrentSlot(XrDockableWindow window)
        {
            if (!TryGetCurrentSlot(window, out XrWindowSlot currentSlot))
            {
                return false;
            }

            Transform windowTransform = window.MovableWindow.transform;
            return windowTransform == GetAnchor(currentSlot)
                || SnapWindowToSlot(window, currentSlot);
        }

        // Scene과 Window ID가 같은 창끼리만 배치 값을 공유하도록 키를 만든다.
        private string GetDockPersistenceKey(string windowId)
        {
            string sceneName = gameObject.scene.name == "StrikerPC"
                ? "StrikerPC-Demo"
                : gameObject.scene.name;
            return $"{DockPersistencePrefix}.{sceneName}.{windowId}";
        }

        // 현재 Scene의 legacy Center/Left처럼 창 자체가 Anchor인 구조를 이동 대상에서 제외한다.
        private bool IsConfiguredAnchor(Transform target)
        {
            if (target == null)
            {
                return false;
            }

            foreach (XrWindowSlot slot in Slots)
            {
                if (GetAnchor(slot) == target)
                {
                    return true;
                }
            }

            return false;
        }

        // Center에는 FPV만, 주변 슬롯에는 FPV가 아닌 창만 등록되도록 검증한다.
        private bool HasCompatibleRoleAndSlot(XrDockableWindow window)
        {
            return !TryGetCurrentSlot(window, out XrWindowSlot slot)
                || (slot == XrWindowSlot.Center) == (window.Role == XrWindowRole.Fpv);
        }

        // Registry 창과 legacy AttachWindow 창을 함께 검사해 슬롯 중복을 막는다.
        private bool IsSlotOccupied(
            XrWindowSlot slot,
            XrDockableWindow allowedDockable,
            Transform allowedLegacy)
        {
            if (TryGetOccupant(slot, out XrDockableWindow dockable)
                && dockable != allowedDockable)
            {
                return true;
            }

            return TryGetLegacyOccupant(slot, out Transform legacy)
                && legacy != allowedLegacy;
        }

        // AttachWindow로 배치한 일반 창이 아직 해당 슬롯에 있는지 확인한다.
        private bool TryGetLegacyOccupant(XrWindowSlot slot, out Transform occupant)
        {
            if (legacyOccupants.TryGetValue(slot, out occupant)
                && occupant != null && occupant.parent == GetAnchor(slot))
            {
                return true;
            }

            legacyOccupants.Remove(slot);
            occupant = null;
            return false;
        }

        // 일반 창이 이전에 점유했던 슬롯 기록을 제거한다.
        private void RemoveLegacyOccupant(Transform target)
        {
            if (target == null)
            {
                return;
            }

            foreach (XrWindowSlot slot in Slots)
            {
                if (legacyOccupants.TryGetValue(slot, out Transform occupant)
                    && occupant == target)
                {
                    legacyOccupants.Remove(slot);
                    return;
                }
            }
        }

        // Registry 초기화 이후에는 드래그 중 계층 검색을 반복하지 않는다.
        private List<XrDockableWindow> GetRegisteredWindows()
        {
            if (!registryInitialized)
            {
                RefreshDockableWindows();
            }

            return registeredWindows;
        }

        // null과 중복 참조를 제외하고 Registry에 추가한다.
        private void AddWindowToRegistry(XrDockableWindow window)
        {
            if (window == null || !window.HasValidIdentity)
            {
                return;
            }
            if (!HasCompatibleRoleAndSlot(window))
            {
                Debug.LogError(
                    $"[XR Dock] Center와 FPV 역할 구성이 일치하지 않습니다: {window.WindowId}",
                    window);
                return;
            }

            foreach (XrDockableWindow registered in registeredWindows)
            {
                if (registered == window)
                {
                    return;
                }
                if (registered != null && string.Equals(
                        registered.WindowId,
                        window.WindowId,
                        StringComparison.Ordinal))
                {
                    Debug.LogError(
                        $"[XR Dock] 중복 Window ID를 등록할 수 없습니다: {window.WindowId}",
                        window);
                    return;
                }
                if (registered != null
                    && window.Role != XrWindowRole.Custom
                    && registered.Role == window.Role)
                {
                    Debug.LogError(
                        $"[XR Dock] 중복 Window 역할을 등록할 수 없습니다: {window.Role}",
                        window);
                    return;
                }
                if (registered != null
                    && TryGetCurrentSlot(registered, out XrWindowSlot registeredSlot)
                    && TryGetCurrentSlot(window, out XrWindowSlot windowSlot)
                    && registeredSlot == windowSlot)
                {
                    Debug.LogError(
                        $"[XR Dock] 슬롯에는 하나의 창만 등록할 수 있습니다: {windowSlot}",
                        window);
                    return;
                }
            }

            RemoveLegacyOccupant(window.MovableWindow.transform);
            registeredWindows.Add(window);
        }
    }
}
