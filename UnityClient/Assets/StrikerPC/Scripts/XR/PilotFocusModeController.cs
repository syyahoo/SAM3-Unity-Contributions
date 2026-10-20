using System.Collections.Generic;
using StrikerPC.Control;
using StrikerPC.Video;
using UnityEngine;

namespace StrikerPC.XR
{
    public enum FocusGestureTransition
    {
        None,
        Entered,
        Exited
    }

    public sealed class FocusGripGestureRecognizer
    {
        private enum GestureState
        {
            WaitingFirstChord,
            FirstChordHeld,
            WaitingSecondChord,
            Focused,
            BlockedUntilRelease
        }

        private readonly float simultaneousSeconds;
        private readonly float firstHoldSeconds;
        private readonly float doubleSqueezeSeconds;
        private GestureState state;
        private bool previousLeft;
        private bool previousRight;
        private float leftPressedAt = -1f;
        private float rightPressedAt = -1f;
        private float firstChordAt;
        private float firstReleasedAt;

        public bool IsFocused => state == GestureState.Focused;

        // 시간 임계값을 고정해 양손 더블 스퀴즈를 판정한다.
        public FocusGripGestureRecognizer(
            float simultaneousSeconds,
            float firstHoldSeconds,
            float doubleSqueezeSeconds)
        {
            this.simultaneousSeconds = Mathf.Max(0.01f, simultaneousSeconds);
            this.firstHoldSeconds = Mathf.Max(this.simultaneousSeconds, firstHoldSeconds);
            this.doubleSqueezeSeconds = Mathf.Max(0.05f, doubleSqueezeSeconds);
        }

        // 양손 그립 표본 하나를 받아 진입 또는 종료 전이만 반환한다.
        public FocusGestureTransition Sample(bool leftHeld, bool rightHeld, float now)
        {
            if (leftHeld && !previousLeft)
            {
                leftPressedAt = now;
            }
            if (rightHeld && !previousRight)
            {
                rightPressedAt = now;
            }
            previousLeft = leftHeld;
            previousRight = rightHeld;

            if (state == GestureState.Focused)
            {
                if (!leftHeld && !rightHeld)
                {
                    Reset();
                    return FocusGestureTransition.Exited;
                }
                return FocusGestureTransition.None;
            }

            if (state == GestureState.BlockedUntilRelease)
            {
                if (!leftHeld && !rightHeld)
                {
                    Reset();
                }
                return FocusGestureTransition.None;
            }

            bool chord = IsChord(leftHeld, rightHeld);
            if (state == GestureState.WaitingFirstChord)
            {
                if (chord)
                {
                    firstChordAt = Mathf.Max(leftPressedAt, rightPressedAt);
                    state = GestureState.FirstChordHeld;
                }
                else if (!leftHeld && !rightHeld)
                {
                    ResetPressTimes();
                }
                return FocusGestureTransition.None;
            }

            if (state == GestureState.FirstChordHeld)
            {
                if (now - firstChordAt > firstHoldSeconds && (leftHeld || rightHeld))
                {
                    state = GestureState.BlockedUntilRelease;
                    return FocusGestureTransition.None;
                }
                if (!leftHeld && !rightHeld)
                {
                    firstReleasedAt = now;
                    state = now - firstChordAt <= firstHoldSeconds
                        ? GestureState.WaitingSecondChord
                        : GestureState.WaitingFirstChord;
                    ResetPressTimes();
                }
                return FocusGestureTransition.None;
            }

            if (now - firstReleasedAt > doubleSqueezeSeconds)
            {
                state = GestureState.WaitingFirstChord;
                if (chord)
                {
                    firstChordAt = Mathf.Max(leftPressedAt, rightPressedAt);
                    state = GestureState.FirstChordHeld;
                }
                return FocusGestureTransition.None;
            }
            if (chord)
            {
                state = GestureState.Focused;
                return FocusGestureTransition.Entered;
            }
            return FocusGestureTransition.None;
        }

        // 실패한 진입이 같은 누름에서 다시 시작되지 않게 양손 해제를 기다린다.
        public void BlockUntilRelease(bool leftHeld, bool rightHeld)
        {
            if (leftHeld || rightHeld)
            {
                state = GestureState.BlockedUntilRelease;
                previousLeft = leftHeld;
                previousRight = rightHeld;
                return;
            }
            Reset();
        }

        private bool IsChord(bool leftHeld, bool rightHeld)
        {
            return leftHeld
                && rightHeld
                && leftPressedAt >= 0f
                && rightPressedAt >= 0f
                && Mathf.Abs(leftPressedAt - rightPressedAt) <= simultaneousSeconds;
        }

        private void Reset()
        {
            state = GestureState.WaitingFirstChord;
            previousLeft = false;
            previousRight = false;
            ResetPressTimes();
        }

        private void ResetPressTimes()
        {
            leftPressedAt = -1f;
            rightPressedAt = -1f;
        }
    }

    public sealed class FocusLossGraceTimer
    {
        private float unavailableSince = -1f;

        // 일시적인 경로 지연은 허용하고 연속 손실만 만료로 판정한다.
        public bool HasExpired(bool available, float now, float graceSeconds)
        {
            if (available)
            {
                Reset();
                return false;
            }
            if (unavailableSince < 0f)
            {
                unavailableSince = now;
                return false;
            }
            return now - unavailableSince >= Mathf.Max(0f, graceSeconds);
        }

        public void Reset()
        {
            unavailableSince = -1f;
        }
    }

    [DisallowMultipleComponent]
    [DefaultExecutionOrder(-90)]
    public sealed class PilotFocusModeController : MonoBehaviour
    {
        [SerializeField] private XrMode2ControlInput controlInput;
        [SerializeField] private XrHudLayout hudLayout;
        [SerializeField] private XrCameraModeSwitcher cameraModeSwitcher;
        [SerializeField] private PilotVideoSourceSelector videoSourceSelector;
        [SerializeField] private PilotSharedFrameTexture sharedFrameSource;
        [SerializeField] private PilotAnnotationOverlay annotationOverlay;
        [SerializeField] private TelloControlOrchestrator controlOrchestrator;
        [SerializeField] private XrMovableWindow[] essentialWindows;
        [SerializeField, Range(0.05f, 0.25f)] private float simultaneousPressSeconds = 0.12f;
        [SerializeField, Range(0.1f, 0.5f)] private float firstSqueezeMaximumSeconds = 0.25f;
        [SerializeField, Range(0.2f, 0.8f)] private float doubleSqueezeWindowSeconds = 0.45f;
        [SerializeField, Min(0.05f)] private float sharedFrameMaximumAgeSeconds = 0.2f;
        [SerializeField, Min(0.1f)] private float sharedFramePreparationTimeoutSeconds = 1f;
        [SerializeField, Min(0.2f)] private float sharedFrameLossGraceSeconds = 1f;
        [SerializeField, Min(0.1f)] private float controlLossGraceSeconds = 0.5f;

        private readonly Dictionary<GameObject, bool> anchorStates = new Dictionary<GameObject, bool>();
        private readonly Dictionary<XrMovableWindow, bool> windowStates =
            new Dictionary<XrMovableWindow, bool>();
        private readonly FocusLossGraceTimer sharedFrameLossTimer = new FocusLossGraceTimer();
        private readonly FocusLossGraceTimer controlLossTimer = new FocusLossGraceTimer();
        private FocusGripGestureRecognizer recognizer;
        private PilotVideoSourceMode previousVideoMode;
        private bool previousPassthrough;
        private bool preparing;
        private bool focused;
        private bool leftGripHeld;
        private bool rightGripHeld;
        private float preparationStartedAt;

        public bool IsFocused => focused;
        public bool IsPreparing => preparing;

        // 비어 있는 참조만 현재 Scene의 기존 컴포넌트로 보완한다.
        private void Awake()
        {
            controlInput ??= FindFirstObjectByType<XrMode2ControlInput>();
            hudLayout ??= FindFirstObjectByType<XrHudLayout>();
            cameraModeSwitcher ??= FindFirstObjectByType<XrCameraModeSwitcher>();
            videoSourceSelector ??= FindFirstObjectByType<PilotVideoSourceSelector>();
            sharedFrameSource ??= FindFirstObjectByType<PilotSharedFrameTexture>();
            annotationOverlay ??= FindFirstObjectByType<PilotAnnotationOverlay>();
            controlOrchestrator ??= FindFirstObjectByType<TelloControlOrchestrator>();
            recognizer = new FocusGripGestureRecognizer(
                simultaneousPressSeconds,
                firstSqueezeMaximumSeconds,
                doubleSqueezeWindowSeconds);
        }

        // 기존 Mode 2 그립 표본을 구독한다.
        private void OnEnable()
        {
            if (controlInput != null)
            {
                controlInput.GripStateSampled += OnGripStateSampled;
            }
        }

        // 준비 중 영상과 포커스 중 필수 경로의 상태를 확인한다.
        private void Update()
        {
            if (preparing)
            {
                if (sharedFrameSource != null
                    && sharedFrameSource.HasFreshFrame(sharedFrameMaximumAgeSeconds))
                {
                    ActivateFocus();
                }
                else if (Time.unscaledTime - preparationStartedAt
                    >= sharedFramePreparationTimeoutSeconds)
                {
                    Debug.LogWarning("[Pilot Focus] 최신 SharedMemory FPV가 없어 진입을 취소했습니다.", this);
                    CancelPreparation(true);
                }
                return;
            }

            if (!focused)
            {
                return;
            }
            float now = Time.unscaledTime;
            bool frameAvailable = sharedFrameSource != null
                && sharedFrameSource.HasFreshFrame(sharedFrameMaximumAgeSeconds);
            if (sharedFrameLossTimer.HasExpired(
                    frameAvailable,
                    now,
                    sharedFrameLossGraceSeconds))
            {
                Debug.LogWarning("[Pilot Focus] SharedMemory FPV가 만료되어 포커스 모드를 종료합니다.", this);
                ExitFocus(true);
                return;
            }
            bool controlAvailable = controlOrchestrator == null || controlOrchestrator.IsConnected;
            if (controlLossTimer.HasExpired(
                    controlAvailable,
                    now,
                    controlLossGraceSeconds))
            {
                Debug.LogWarning("[Pilot Focus] 사용할 수 있는 조종 경로가 없어 포커스 모드를 종료합니다.", this);
                ExitFocus(true);
            }
        }

        // 모든 임시 상태를 원래대로 되돌린다.
        private void OnDisable()
        {
            if (controlInput != null)
            {
                controlInput.GripStateSampled -= OnGripStateSampled;
            }
            ExitFocus(false);
        }

        // 앱 포커스를 잃으면 숨긴 안전 정보를 즉시 복원한다.
        private void OnApplicationFocus(bool hasFocus)
        {
            if (ShouldExitForApplicationFocus(hasFocus, Application.isEditor))
            {
                ExitFocus(true);
            }
        }

        // Editor PCVR 검증 중 데스크톱 창 전환은 모드를 해제하지 않는다.
        private static bool ShouldExitForApplicationFocus(bool hasFocus, bool isEditor)
        {
            return !hasFocus && !isEditor;
        }

        // 더블 스퀴즈 상태 전이만 포커스 진입·종료로 변환한다.
        private void OnGripStateSampled(bool leftHeld, bool rightHeld)
        {
            leftGripHeld = leftHeld;
            rightGripHeld = rightHeld;
            FocusGestureTransition transition = recognizer.Sample(
                leftHeld,
                rightHeld,
                Time.unscaledTime);
            if (transition == FocusGestureTransition.Entered)
            {
                BeginPreparation();
            }
            else if (transition == FocusGestureTransition.Exited)
            {
                ExitFocus(false);
            }
        }

        // 영상 소스만 먼저 SharedMemory로 전환하고 최신 프레임을 기다린다.
        private void BeginPreparation()
        {
            if (preparing || focused || videoSourceSelector == null || sharedFrameSource == null)
            {
                return;
            }
            previousVideoMode = videoSourceSelector.SourceMode;
            videoSourceSelector.SetSourceMode(PilotVideoSourceMode.SharedMemory);
            preparing = true;
            preparationStartedAt = Time.unscaledTime;
            if (sharedFrameSource.HasFreshFrame(sharedFrameMaximumAgeSeconds))
            {
                ActivateFocus();
            }
        }

        // 창·주석·Passthrough를 스냅샷한 뒤 필수 화면만 남긴다.
        private void ActivateFocus()
        {
            if (!preparing)
            {
                return;
            }
            preparing = false;
            SnapshotAndApplyWindows();
            previousPassthrough = cameraModeSwitcher != null
                && cameraModeSwitcher.IsPassthroughEnabled;
            cameraModeSwitcher?.SetPassthroughEnabled(false);
            annotationOverlay?.SetFocusSuppressed(true);
            controlOrchestrator?.RequestTemporarySharedMemory();
            sharedFrameLossTimer.Reset();
            controlLossTimer.Reset();
            focused = true;
        }

        // 준비 단계만 취소하고 원래 영상 소스를 복원한다.
        private void CancelPreparation(bool blockUntilRelease)
        {
            if (!preparing)
            {
                return;
            }
            preparing = false;
            sharedFrameLossTimer.Reset();
            controlLossTimer.Reset();
            videoSourceSelector?.SetSourceMode(previousVideoMode);
            if (blockUntilRelease)
            {
                recognizer.BlockUntilRelease(leftGripHeld, rightGripHeld);
            }
        }

        // 포커스에서 변경한 모든 상태와 기본 제어 경로를 복원한다.
        private void ExitFocus(bool blockUntilRelease)
        {
            if (preparing)
            {
                CancelPreparation(blockUntilRelease);
            }
            if (!focused)
            {
                if (blockUntilRelease)
                {
                    recognizer?.BlockUntilRelease(leftGripHeld, rightGripHeld);
                }
                return;
            }

            focused = false;
            sharedFrameLossTimer.Reset();
            controlLossTimer.Reset();
            controlOrchestrator?.RestoreConfiguredControlMode();
            annotationOverlay?.SetFocusSuppressed(false);
            cameraModeSwitcher?.SetPassthroughEnabled(previousPassthrough);
            videoSourceSelector?.SetSourceMode(previousVideoMode);
            RestoreWindows();
            if (blockUntilRelease)
            {
                recognizer.BlockUntilRelease(leftGripHeld, rightGripHeld);
            }
        }

        // XR 슬롯과 각 창의 활성 상태를 저장하고 필수 창만 표시한다.
        private void SnapshotAndApplyWindows()
        {
            anchorStates.Clear();
            if (hudLayout != null)
            {
                foreach (XrWindowSlot slot in System.Enum.GetValues(typeof(XrWindowSlot)))
                {
                    Transform anchor = hudLayout.GetAnchor(slot);
                    if (anchor == null)
                    {
                        continue;
                    }
                    anchorStates[anchor.gameObject] = anchor.gameObject.activeSelf;
                    bool visible = slot is XrWindowSlot.Center or XrWindowSlot.Upper;
                    anchor.gameObject.SetActive(visible);
                }
            }

            HashSet<XrMovableWindow> essential = new HashSet<XrMovableWindow>(
                essentialWindows ?? System.Array.Empty<XrMovableWindow>());
            windowStates.Clear();
            XrMovableWindow[] windows = FindObjectsByType<XrMovableWindow>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
            foreach (XrMovableWindow window in windows)
            {
                windowStates[window] = window.gameObject.activeSelf;
                if (essential.Contains(window))
                {
                    window.gameObject.SetActive(true);
                    window.SetFocusPresentation(true);
                }
                else
                {
                    window.gameObject.SetActive(false);
                }
            }
        }

        // 임시 표시를 해제하고 창과 Anchor의 기존 활성 상태를 복원한다.
        private void RestoreWindows()
        {
            foreach (KeyValuePair<XrMovableWindow, bool> item in windowStates)
            {
                if (item.Key == null)
                {
                    continue;
                }
                item.Key.SetFocusPresentation(false);
                item.Key.gameObject.SetActive(item.Value);
            }
            foreach (KeyValuePair<GameObject, bool> item in anchorStates)
            {
                if (item.Key != null)
                {
                    item.Key.SetActive(item.Value);
                }
            }
            windowStates.Clear();
            anchorStates.Clear();
        }
    }
}
