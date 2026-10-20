using System;
using System.Collections.Generic;
using StrikerPC.State;
using StrikerPC.XR;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;
using UnityEngine.XR.Interaction.Toolkit.Inputs.Readers;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

namespace StrikerPC.Control
{
    public static class PrecisionFlightInputProfile
    {
        // Focus Mode 입력을 저속 한계와 Expo 곡선에 맞춰 정밀 조종값으로 변환한다.
        public static Vector4 Apply(
            Vector4 axes,
            bool enabled,
            float horizontalLimit,
            float verticalLimit,
            float yawLimit,
            float exponent)
        {
            if (!enabled)
            {
                return axes;
            }

            return new Vector4(
                ShapeAxis(axes.x, horizontalLimit, exponent),
                ShapeAxis(axes.y, horizontalLimit, exponent),
                ShapeAxis(axes.z, verticalLimit, exponent),
                ShapeAxis(axes.w, yawLimit, exponent));
        }

        // 축 부호를 유지하면서 중앙 입력을 완만하게 만들고 최대 출력을 제한한다.
        private static float ShapeAxis(float value, float limit, float exponent)
        {
            float magnitude = Mathf.Pow(
                Mathf.Clamp01(Mathf.Abs(value)),
                Mathf.Max(1f, exponent));
            return Mathf.Sign(value) * magnitude * Mathf.Clamp01(limit);
        }
    }

    [DisallowMultipleComponent]
    [DefaultExecutionOrder(-100)]
    [RequireComponent(typeof(TelloControlOrchestrator))]
    public sealed class XrMode2ControlInput : MonoBehaviour
    {
        [SerializeField] private TelloControlOrchestrator orchestrator;
        [SerializeField] private InputActionProperty leftStickAction;
        [SerializeField] private InputActionProperty rightStickAction;
        [SerializeField] private InputActionProperty deadmanAction;
        [SerializeField] private InputActionProperty rightDeadmanAction;
        [SerializeField] private InputActionProperty flightToggleAction;
        [SerializeField, Range(0f, 0.5f)] private float stickDeadzone = 0.12f;
        [SerializeField, Min(0.25f)] private float flightToggleHoldSeconds = 1f;
        [SerializeField] private PilotFocusModeController focusModeController;
        [SerializeField, Range(0.1f, 1f)] private float focusHorizontalAxisLimit = 0.45f;
        [SerializeField, Range(0.1f, 1f)] private float focusVerticalAxisLimit = 0.35f;
        [SerializeField, Range(0.1f, 1f)] private float focusYawAxisLimit = 0.35f;
        [SerializeField, Range(1f, 3f)] private float focusInputExponent = 1.5f;
        [SerializeField] private TelloStateReceiver stateReceiver;
        [SerializeField, Range(1f, 45f)] private float maximumTiltDegrees = 25f;
        [SerializeField, Range(0.8f, 1f)] private float maximumStickThreshold = 0.98f;
        [SerializeField, Range(0f, 1f)] private float shortHapticAmplitude = 0.35f;
        [SerializeField, Range(0f, 1f)] private float weakHapticAmplitude = 0.18f;
        [SerializeField, Range(0f, 1f)] private float delayHapticAmplitude = 0.3f;
        [SerializeField, Range(0f, 1f)] private float strongHapticAmplitude = 0.9f;
        [SerializeField, Min(0.1f)] private float delayHapticIntervalSeconds = 0.6f;

        private bool leftEnabledByThis;
        private bool rightEnabledByThis;
        private bool deadmanEnabledByThis;
        private bool rightDeadmanEnabledByThis;
        private bool flightToggleEnabledByThis;
        private bool deadmanHeld;
        private bool leftGripWasHeld;
        private bool rightGripWasHeld;
        private float flightTogglePressedAt = -1f;
        private bool flightToggleConsumed;
        private bool maximumTiltReached;
        private bool failsafeActive;
        private float nextDelayHapticAt;
        private readonly List<UiScrollGate> uiScrollGates = new List<UiScrollGate>();

        public event Action<bool, bool> GripStateSampled;
        public Vector4 CurrentControlAxes { get; private set; }

        // 기존 참조를 유지하고 비어 있는 XR Action만 기본값으로 만든다.
        private void Awake()
        {
            if (orchestrator == null)
            {
                orchestrator = GetComponent<TelloControlOrchestrator>();
            }

            if (stateReceiver == null)
            {
                stateReceiver = FindFirstObjectByType<TelloStateReceiver>();
            }

            if (focusModeController == null)
            {
                focusModeController = FindFirstObjectByType<PilotFocusModeController>();
            }

            EnsureDefaultActions();
        }

        // Mode 2 입력 Action을 활성화한다.
        private void OnEnable()
        {
            EnsureDefaultActions();
            EnableAction(leftStickAction.action, ref leftEnabledByThis);
            EnableAction(rightStickAction.action, ref rightEnabledByThis);
            EnableAction(deadmanAction.action, ref deadmanEnabledByThis);
            EnableAction(rightDeadmanAction.action, ref rightDeadmanEnabledByThis);
            EnableAction(flightToggleAction.action, ref flightToggleEnabledByThis);
            InstallUiScrollGates();
        }

        // 다른 HUD 컴포넌트의 Awake가 끝난 뒤 텔레메트리 수신기를 다시 찾는다.
        private void Start()
        {
            if (stateReceiver == null)
            {
                stateReceiver = FindFirstObjectByType<TelloStateReceiver>();
            }

            InstallUiScrollGates();
        }

        // 매 프레임 최신 조이스틱 입력을 조종 컴포넌트에 전달한다.
        private void Update()
        {
            if (orchestrator == null)
            {
                CurrentControlAxes = Vector4.zero;
                return;
            }

            bool leftGripIsHeld = IsGripHeld(InteractorHandedness.Left);
            bool rightGripIsHeld = IsGripHeld(InteractorHandedness.Right);
            GripStateSampled?.Invoke(leftGripIsHeld, rightGripIsHeld);
            UpdateFlightToggle(rightGripIsHeld);
            UpdateSafetyHaptics();
            UpdateInteractorGates(leftGripIsHeld, rightGripIsHeld);
            UpdateGripHaptics(leftGripIsHeld, rightGripIsHeld);

            if (!HasActiveGrip(leftGripIsHeld, rightGripIsHeld))
            {
                if (deadmanHeld)
                {
                    ApplyNeutral();
                }

                return;
            }

            Vector2 leftStick = ApplyDeadzone(FilterStickByGrip(
                leftStickAction.action?.ReadValue<Vector2>() ?? Vector2.zero,
                leftGripIsHeld));
            Vector2 rightStick = ApplyDeadzone(FilterStickByGrip(
                rightStickAction.action?.ReadValue<Vector2>() ?? Vector2.zero,
                rightGripIsHeld));
            Vector4 axes = MapMode2Axes(leftStick, rightStick);
            axes = PrecisionFlightInputProfile.Apply(
                axes,
                focusModeController != null && focusModeController.IsFocused,
                focusHorizontalAxisLimit,
                focusVerticalAxisLimit,
                focusYawAxisLimit,
                focusInputExponent);

            CurrentControlAxes = axes;
            orchestrator.SetControlAxes(axes.x, axes.y, axes.z, axes.w);
            if (!deadmanHeld)
            {
                orchestrator.SetDeadman(true);
                deadmanHeld = true;
            }

            UpdateMaximumTiltHaptic(leftStick, rightStick);
        }

        // 입력 Action과 조종 값을 안전하게 중립 상태로 되돌린다.
        private void OnDisable()
        {
            GripStateSampled?.Invoke(false, false);
            RestoreUiScrollGates();
            ApplyNeutral();
            DisableOwnedAction(leftStickAction.action, ref leftEnabledByThis);
            DisableOwnedAction(rightStickAction.action, ref rightEnabledByThis);
            DisableOwnedAction(deadmanAction.action, ref deadmanEnabledByThis);
            DisableOwnedAction(rightDeadmanAction.action, ref rightDeadmanEnabledByThis);
            DisableOwnedAction(flightToggleAction.action, ref flightToggleEnabledByThis);
            ResetFlightToggle();
            failsafeActive = false;
            nextDelayHapticAt = 0f;
        }

        // 오른손 B 롱프레스로 지상에서는 이륙하고 비행 중에는 착륙한다.
        private void UpdateFlightToggle(bool rightGripIsHeld)
        {
            bool flightToggleIsHeld = flightToggleAction.action?.IsPressed() == true;
            if (!CanUseFlightToggle(rightGripIsHeld, flightToggleIsHeld))
            {
                ResetFlightToggle();
                return;
            }

            float now = Time.unscaledTime;
            if (flightTogglePressedAt < 0f)
            {
                flightTogglePressedAt = now;
                return;
            }

            if (!HasReachedHoldDuration(
                    flightTogglePressedAt,
                    now,
                    flightToggleHoldSeconds,
                    flightToggleConsumed))
            {
                return;
            }

            flightToggleConsumed = true;
            if (!orchestrator.IsConnected)
            {
                Debug.LogWarning("[Drone Control] 조종 경로가 연결되지 않아 이륙/착륙 입력을 무시했습니다.", this);
                return;
            }

            if (orchestrator.IsAirborne)
            {
                orchestrator.RequestLand();
                SendHapticToBothHands(strongHapticAmplitude, 0.3f);
            }
            else
            {
                orchestrator.RequestTakeoff();
            }
        }

        // 오른손 그립과 비행 버튼을 함께 누른 경우에만 이륙·착륙을 허용한다.
        private static bool CanUseFlightToggle(bool rightGripIsHeld, bool flightToggleIsHeld)
        {
            return rightGripIsHeld && flightToggleIsHeld;
        }

        // 버튼을 놓으면 다음 롱프레스 입력을 받을 수 있게 초기화한다.
        private void ResetFlightToggle()
        {
            flightTogglePressedAt = -1f;
            flightToggleConsumed = false;
        }

        // 롱프레스 시간이 충족됐고 아직 처리하지 않았는지 확인한다.
        private static bool HasReachedHoldDuration(
            float pressedAt,
            float now,
            float requiredSeconds,
            bool consumed)
        {
            return !consumed && now - pressedAt >= requiredSeconds;
        }

        // Mode 2 스틱 축을 Tello RC 축 순서로 변환한다.
        private static Vector4 MapMode2Axes(Vector2 leftStick, Vector2 rightStick)
        {
            return new Vector4(
                rightStick.x,
                rightStick.y,
                leftStick.y,
                leftStick.x);
        }

        // 그립을 누른 손의 조이스틱 값만 드론 조종에 사용한다.
        private static Vector2 FilterStickByGrip(Vector2 value, bool gripIsHeld)
        {
            return gripIsHeld ? value : Vector2.zero;
        }

        // 한 손 이상 그립을 누르고 있을 때만 데드맨을 활성화한다.
        private static bool HasActiveGrip(bool leftGripIsHeld, bool rightGripIsHeld)
        {
            return leftGripIsHeld || rightGripIsHeld;
        }

        // 스틱 입력 또는 기체 자세가 설정된 최대 기울기에 도달했는지 확인한다.
        private void UpdateMaximumTiltHaptic(Vector2 leftStick, Vector2 rightStick)
        {
            bool reached = HasReachedMaximumInput(leftStick, rightStick, maximumStickThreshold);
            if (stateReceiver != null)
            {
                TelloConnectionState state = stateReceiver.Current;
                reached |= state.stateConnected
                    && (Mathf.Abs(state.pitch) >= maximumTiltDegrees
                        || Mathf.Abs(state.roll) >= maximumTiltDegrees);
            }

            if (reached && !maximumTiltReached)
            {
                SendHapticToBothHands(weakHapticAmplitude, 0.06f);
            }

            maximumTiltReached = reached;
        }

        // 통신 지연과 비행 중 Failsafe 진입을 햅틱 패턴으로 알린다.
        private void UpdateSafetyHaptics()
        {
            bool delayed = IsCommunicationDelayed();
            bool currentFailsafe = delayed && orchestrator.IsAirborne;
            if (currentFailsafe && !failsafeActive)
            {
                SendHapticToBothHands(strongHapticAmplitude, 0.3f);
                nextDelayHapticAt = Time.unscaledTime + delayHapticIntervalSeconds;
            }

            failsafeActive = currentFailsafe;
            if (!delayed)
            {
                nextDelayHapticAt = 0f;
                return;
            }

            if ((deadmanHeld || orchestrator.IsAirborne) && Time.unscaledTime >= nextDelayHapticAt)
            {
                SendHapticToBothHands(delayHapticAmplitude, 0.09f);
                nextDelayHapticAt = Time.unscaledTime + delayHapticIntervalSeconds;
            }
        }

        // 제어 전송 실패 또는 수신 중이던 텔레메트리 단절을 통신 지연으로 판정한다.
        private bool IsCommunicationDelayed()
        {
            if (!orchestrator.IsConnected || orchestrator.ConsecutiveFailedFrames > 0)
            {
                return true;
            }

            if (stateReceiver == null)
            {
                return false;
            }

            TelloConnectionState state = stateReceiver.Current;
            return state.receivedUtcMilliseconds > 0 && !state.stateConnected;
        }

        // 양쪽 스틱 중 하나가 최대 입력 임계값에 도달했는지 확인한다.
        private static bool HasReachedMaximumInput(Vector2 leftStick, Vector2 rightStick, float threshold)
        {
            float squaredThreshold = threshold * threshold;
            return leftStick.sqrMagnitude >= squaredThreshold
                || rightStick.sqrMagnitude >= squaredThreshold;
        }

        // 해당 손의 그립을 누르는 동안 조이스틱의 UI 스크롤 값을 차단한다.
        private static Vector2 FilterUiScrollValue(Vector2 value, bool gripIsHeld)
        {
            return gripIsHeld ? Vector2.zero : value;
        }

        // 현재 Scene의 양손 XR 인터랙터에 UI 스크롤 차단기를 연결한다.
        private void InstallUiScrollGates()
        {
            NearFarInteractor[] nearFarInteractors = FindObjectsByType<NearFarInteractor>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
            foreach (NearFarInteractor interactor in nearFarInteractors)
            {
                TryInstallUiScrollGate(
                    interactor,
                    interactor.handedness,
                    interactor.uiScrollInput);
            }

            XRRayInteractor[] rayInteractors = FindObjectsByType<XRRayInteractor>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
            foreach (XRRayInteractor interactor in rayInteractors)
            {
                TryInstallUiScrollGate(
                    interactor,
                    interactor.handedness,
                    interactor.uiScrollInput);
            }
        }

        // 아직 처리하지 않은 XR 스크롤 입력에만 차단기를 연결한다.
        private void TryInstallUiScrollGate(
            Behaviour interactor,
            InteractorHandedness handedness,
            XRInputValueReader<Vector2> scrollInput)
        {
            if (interactor == null || scrollInput == null)
            {
                return;
            }

            foreach (UiScrollGate gate in uiScrollGates)
            {
                if (ReferenceEquals(gate.Source, scrollInput))
                {
                    return;
                }
            }

            UiScrollGate newGate = new UiScrollGate(
                this,
                interactor,
                handedness,
                scrollInput);
            newGate.Install();
            uiScrollGates.Add(newGate);
        }

        // 그립을 누른 손은 드론 전용으로 전환하고 반대 손 XR 조작은 유지한다.
        private void UpdateInteractorGates(bool leftGripIsHeld, bool rightGripIsHeld)
        {
            foreach (UiScrollGate gate in uiScrollGates)
            {
                bool gripIsHeld = gate.Handedness switch
                {
                    InteractorHandedness.Left => leftGripIsHeld,
                    InteractorHandedness.Right => rightGripIsHeld,
                    _ => false
                };
                gate.SetDroneControlActive(gripIsHeld);
            }
        }

        // 컴포넌트가 꺼질 때 기존 XR 스크롤 입력 경로를 복원한다.
        private void RestoreUiScrollGates()
        {
            foreach (UiScrollGate gate in uiScrollGates)
            {
                gate.Restore();
            }

            uiScrollGates.Clear();
        }

        // 지정한 손의 그립 입력 상태를 반환한다.
        private bool IsGripHeld(InteractorHandedness handedness)
        {
            return handedness switch
            {
                InteractorHandedness.Left => deadmanAction.action?.IsPressed() == true,
                InteractorHandedness.Right => rightDeadmanAction.action?.IsPressed() == true,
                _ => false
            };
        }

        // 새로 그립을 누른 손에 조종 활성화 햅틱을 보낸다.
        private void UpdateGripHaptics(bool leftGripIsHeld, bool rightGripIsHeld)
        {
            if (leftGripIsHeld && !leftGripWasHeld)
            {
                SendHaptic(CommonUsages.LeftHand, shortHapticAmplitude, 0.08f);
            }

            if (rightGripIsHeld && !rightGripWasHeld)
            {
                SendHaptic(CommonUsages.RightHand, shortHapticAmplitude, 0.08f);
            }

            leftGripWasHeld = leftGripIsHeld;
            rightGripWasHeld = rightGripIsHeld;
        }

        // 양쪽 XR 컨트롤러에 동일한 햅틱을 보낸다.
        private static void SendHapticToBothHands(float amplitude, float duration)
        {
            SendHaptic(CommonUsages.LeftHand, amplitude, duration);
            SendHaptic(CommonUsages.RightHand, amplitude, duration);
        }

        // 지정한 손의 햅틱 지원 컨트롤러에 진동을 보낸다.
        private static void SendHaptic(UnityEngine.InputSystem.Utilities.InternedString hand, float amplitude, float duration)
        {
            XRControllerWithRumble controller = InputSystem.GetDevice<XRControllerWithRumble>(hand);
            controller?.SendImpulse(Mathf.Clamp01(amplitude), Mathf.Max(0f, duration));
        }

        // 작은 스틱 흔들림을 제거하고 나머지 입력 범위를 다시 정규화한다.
        private Vector2 ApplyDeadzone(Vector2 value)
        {
            float magnitude = value.magnitude;
            if (magnitude <= stickDeadzone)
            {
                return Vector2.zero;
            }

            float scaledMagnitude = Mathf.Clamp01(
                (magnitude - stickDeadzone) / (1f - stickDeadzone));
            return value.normalized * scaledMagnitude;
        }

        // 비어 있는 Action에 Quest Controller 기본 바인딩을 지정한다.
        private void EnsureDefaultActions()
        {
            if (HasNoBindings(leftStickAction))
            {
                leftStickAction = CreateAction(
                    "Mode 2 Left Stick",
                    InputActionType.Value,
                    "<XRController>{LeftHand}/primary2DAxis",
                    "Vector2");
            }

            if (HasNoBindings(rightStickAction))
            {
                rightStickAction = CreateAction(
                    "Mode 2 Right Stick",
                    InputActionType.Value,
                    "<XRController>{RightHand}/primary2DAxis",
                    "Vector2");
            }

            if (HasNoBindings(deadmanAction))
            {
                deadmanAction = CreateAction(
                    "Mode 2 Left Grip",
                    InputActionType.Button,
                    "<XRController>{LeftHand}/gripPressed");
            }

            if (HasNoBindings(rightDeadmanAction))
            {
                rightDeadmanAction = CreateAction(
                    "Mode 2 Right Grip",
                    InputActionType.Button,
                    "<XRController>{RightHand}/gripPressed");
            }

            if (HasNoBindings(flightToggleAction))
            {
                flightToggleAction = CreateAction(
                    "Flight Toggle",
                    InputActionType.Button,
                    "<XRController>{RightHand}/secondaryButton");
            }
        }

        // Unity가 직렬화한 빈 Action도 기본 바인딩 대상으로 판별한다.
        private static bool HasNoBindings(InputActionProperty property)
        {
            return property.action == null || property.action.bindings.Count == 0;
        }

        // 런타임 내장 Input Action을 만든다.
        private static InputActionProperty CreateAction(
            string name,
            InputActionType type,
            string binding,
            string expectedControlType = null)
        {
            return new InputActionProperty(
                new InputAction(name, type, binding, expectedControlType: expectedControlType));
        }

        // 비활성 Action만 이 컴포넌트가 직접 활성화한다.
        private static void EnableAction(InputAction action, ref bool enabledByThis)
        {
            if (action == null || action.enabled)
            {
                return;
            }

            action.Enable();
            enabledByThis = true;
        }

        // 이 컴포넌트가 활성화한 Action만 비활성화한다.
        private static void DisableOwnedAction(InputAction action, ref bool enabledByThis)
        {
            if (!enabledByThis)
            {
                return;
            }

            action?.Disable();
            enabledByThis = false;
        }

        // 조종 입력과 데드맨을 즉시 중립화한다.
        private void ApplyNeutral()
        {
            CurrentControlAxes = Vector4.zero;
            if (orchestrator != null)
            {
                orchestrator.SetControlAxes(0f, 0f, 0f, 0f);
                orchestrator.SetDeadman(false);
            }

            deadmanHeld = false;
            leftGripWasHeld = false;
            rightGripWasHeld = false;
            maximumTiltReached = false;
        }

        private sealed class UiScrollGate : IXRInputValueReader<Vector2>
        {
            private readonly XrMode2ControlInput owner;
            private readonly Behaviour interactor;
            private readonly InteractorHandedness handedness;
            private readonly XRInputValueReader<Vector2> source;
            private readonly IXRInputValueReader<Vector2> previousBypass;
            private bool interactorDisabledByThis;

            public XRInputValueReader<Vector2> Source => source;
            public InteractorHandedness Handedness => handedness;

            // 기존 입력 경로를 보존하는 UI 스크롤 차단기를 만든다.
            public UiScrollGate(
                XrMode2ControlInput owner,
                Behaviour interactor,
                InteractorHandedness handedness,
                XRInputValueReader<Vector2> source)
            {
                this.owner = owner;
                this.interactor = interactor;
                this.handedness = handedness;
                this.source = source;
                previousBypass = source.bypass;
            }

            // XR 스크롤 입력의 런타임 우회 경로에 차단기를 연결한다.
            public void Install()
            {
                source.bypass = this;
            }

            // 다른 시스템이 경로를 바꾸지 않았다면 원래 입력 경로를 복원한다.
            public void Restore()
            {
                SetDroneControlActive(false);
                if (ReferenceEquals(source.bypass, this))
                {
                    source.bypass = previousBypass;
                }
            }

            // 그립을 누른 손의 XR 인터랙터만 일시적으로 비활성화한다.
            public void SetDroneControlActive(bool isActive)
            {
                if (isActive)
                {
                    if (interactor.enabled)
                    {
                        interactor.enabled = false;
                        interactorDisabledByThis = true;
                    }

                    return;
                }

                if (interactorDisabledByThis)
                {
                    interactor.enabled = true;
                    interactorDisabledByThis = false;
                }
            }

            // 데드맨 상태에 따라 원래 UI 스크롤 값을 통과시키거나 차단한다.
            public Vector2 ReadValue()
            {
                Vector2 value = previousBypass?.ReadValue() ?? source.ReadValue();
                return FilterUiScrollValue(value, owner.IsGripHeld(handedness));
            }

            // 데드맨 중에는 UI 스크롤 입력이 없는 것으로 처리한다.
            public bool TryReadValue(out Vector2 value)
            {
                bool hasValue = previousBypass != null
                    ? previousBypass.TryReadValue(out value)
                    : source.TryReadValue(out value);
                bool gripIsHeld = owner.IsGripHeld(handedness);
                value = FilterUiScrollValue(value, gripIsHeld);
                return !gripIsHeld && hasValue;
            }
        }
    }
}
