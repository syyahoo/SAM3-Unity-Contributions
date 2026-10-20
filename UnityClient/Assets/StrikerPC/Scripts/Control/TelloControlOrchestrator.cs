using System;
using System.Diagnostics;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace StrikerPC.Control
{
    public sealed class TelloControlOrchestrator : MonoBehaviour
    {
        private const int PythonRelayInitialRetryDelayMilliseconds = 250;
        private const int PythonRelayMaximumRetryDelayMilliseconds = 2000;
        private const int ConnectionRetryPollMilliseconds = 50;

        [SerializeField] private TelloControlSettings settings;
        [SerializeField] private bool connectOnEnable = true;
        [SerializeField] private bool collectTimingSamples;

        private Thread senderThread;
        private Thread transitionThread;
        private IDroneControlTransport transport;
        private IDroneControlTransport standbyTransport;
        private readonly object transportGate = new object();
        private volatile bool senderRunning;
        private DroneControlMode activeMode;
        private DroneControlMode standbyMode;
        private int temporarySharedRequested;
        private int transitionGeneration;
        private int pendingModeChange = -1;
        private int pendingSharedFallback;
        private int pendingFallbackNeutral;
        private int leftRight;
        private int forwardBack;
        private int upDown;
        private int yaw;
        private int deadmanHeld;
        private int reconnectReleaseRequired;
        private int pendingCommand;
        private int airborne;
        private long latestInputTicks;
        private long sentFrames;
        private long failedFrames;
        private int consecutiveFailedFrames;
        private long lastInputToSendMicroseconds;
        private long lastScheduleJitterMicroseconds;
        private string pendingError;
        private ulong sequence;
        private long lastMeasuredInputTicks;
        private readonly ControlTimingWindow timingWindow = new ControlTimingWindow();

        public event Action<DroneControlMode> ActiveModeChanged;
        public event Action SharedMemoryFallback;

        public DroneControlMode ActiveMode
        {
            get
            {
                lock (transportGate)
                {
                    return activeMode;
                }
            }
        }

        public bool IsConnected
        {
            get
            {
                lock (transportGate)
                {
                    return transport?.IsConnected == true;
                }
            }
        }
        public bool IsAirborne => Volatile.Read(ref airborne) != 0;
        public long SentFrames => Interlocked.Read(ref sentFrames);
        public long FailedFrames => Interlocked.Read(ref failedFrames);
        public int ConsecutiveFailedFrames => Volatile.Read(ref consecutiveFailedFrames);
        public long LastInputToSendMicroseconds => Interlocked.Read(ref lastInputToSendMicroseconds);
        public long LastScheduleJitterMicroseconds => Interlocked.Read(ref lastScheduleJitterMicroseconds);

        // 현재 A/B 측정의 p95와 실패율을 반환한다.
        public ControlTimingSummary CaptureTimingSummary()
        {
            return timingWindow.Capture(SentFrames, FailedFrames);
        }

        // A/B 측정 표본과 송신 카운터를 초기화한다.
        [ContextMenu("Control/Reset Timing Metrics")]
        public void ResetTimingMetrics()
        {
            timingWindow.Reset();
            Interlocked.Exchange(ref sentFrames, 0);
            Interlocked.Exchange(ref failedFrames, 0);
            Interlocked.Exchange(ref lastMeasuredInputTicks, 0);
        }

        // 현재 A/B 측정 결과를 Console에 출력한다.
        [ContextMenu("Control/Log Timing Metrics")]
        public void LogTimingMetrics()
        {
            Debug.Log($"[Drone Control] {activeMode}: {CaptureTimingSummary()}", this);
        }

        // 설정된 조종 경로를 시작한다.
        private void OnEnable()
        {
            if (connectOnEnable)
            {
                StartControl();
            }
        }

        // 백그라운드 오류를 Unity 메인 스레드에 출력한다.
        private void Update()
        {
            if (!string.IsNullOrEmpty(pendingError))
            {
                string message = pendingError;
                pendingError = null;
                Debug.LogWarning($"[Drone Control] {message}", this);
            }

            int changedMode = Interlocked.Exchange(ref pendingModeChange, -1);
            if (changedMode >= 0)
            {
                ActiveModeChanged?.Invoke((DroneControlMode)changedMode);
            }

            if (Interlocked.Exchange(ref pendingSharedFallback, 0) != 0)
            {
                SharedMemoryFallback?.Invoke();
            }
        }

        // 포커스를 잃으면 즉시 다음 송신부터 중립을 적용한다.
        private void OnApplicationFocus(bool hasFocus)
        {
            if (!hasFocus)
            {
                ApplyNeutralInput();
            }
        }

        // 앱이 중지되면 즉시 다음 송신부터 중립을 적용한다.
        private void OnApplicationPause(bool pauseStatus)
        {
            if (pauseStatus)
            {
                ApplyNeutralInput();
            }
        }

        // 중립 명령 후 조종 경로를 종료한다.
        private void OnDisable()
        {
            StopControl();
        }

        // 현재 설정의 전송 스레드를 시작한다.
        public bool StartControl()
        {
            if (senderRunning)
            {
                return true;
            }

            if (settings == null)
            {
                Debug.LogError("[Drone Control] TelloControlSettings가 연결되지 않았습니다.", this);
                return false;
            }

            activeMode = settings.ControlMode;
            senderRunning = true;
            senderThread = new Thread(SenderLoop)
            {
                IsBackground = true,
                Name = $"TelloControl-{activeMode}"
            };
            senderThread.Start();
            return true;
        }

        // 착륙 상태에서만 전송 경로를 교체한다.
        public bool TrySwitchMode(DroneControlMode requestedMode)
        {
            if (IsAirborne || requestedMode == ActiveMode
                || Volatile.Read(ref temporarySharedRequested) != 0)
            {
                return false;
            }

            if (!StopControl())
            {
                return false;
            }

            activeMode = requestedMode;
            senderRunning = true;
            senderThread = new Thread(SenderLoop)
            {
                IsBackground = true,
                Name = $"TelloControl-{activeMode}"
            };
            senderThread.Start();
            return true;
        }

        // PythonRelay를 유지한 채 공유메모리 제어 경로를 백그라운드에서 준비한다.
        public bool RequestTemporarySharedMemory()
        {
            if (settings == null || settings.ControlMode != DroneControlMode.PythonRelay)
            {
                return false;
            }

            lock (transportGate)
            {
                if (activeMode == DroneControlMode.SharedMemory)
                {
                    return true;
                }
                if (transport == null || activeMode != DroneControlMode.PythonRelay)
                {
                    return false;
                }
            }

            if (Interlocked.Exchange(ref temporarySharedRequested, 1) == 1)
            {
                return true;
            }

            int generation = Interlocked.Increment(ref transitionGeneration);
            transitionThread = new Thread(() => PrepareSharedMemoryTransport(generation))
            {
                IsBackground = true,
                Name = "TelloControl-SharedMemory-Handoff"
            };
            transitionThread.Start();
            return true;
        }

        // 포커스 모드가 끝나면 standby PythonRelay를 즉시 활성 경로로 복원한다.
        public void RestoreConfiguredControlMode()
        {
            Interlocked.Exchange(ref temporarySharedRequested, 0);
            Interlocked.Increment(ref transitionGeneration);
            IDroneControlTransport sharedToDispose = null;
            lock (transportGate)
            {
                if (activeMode != DroneControlMode.SharedMemory || standbyTransport == null)
                {
                    return;
                }

                sharedToDispose = transport;
                transport = standbyTransport;
                activeMode = standbyMode;
                standbyTransport = null;
                Interlocked.Exchange(ref pendingModeChange, (int)activeMode);
            }
            sharedToDispose?.Dispose();
        }

        // 정규화된 4축 조종 입력을 최신 상태로 저장한다.
        public void SetControlAxes(float leftRightValue, float forwardBackValue, float upDownValue, float yawValue)
        {
            int maximum = settings != null ? settings.MaximumAxis : 100;
            Volatile.Write(ref leftRight, QuantizeAxis(leftRightValue, maximum));
            Volatile.Write(ref forwardBack, QuantizeAxis(forwardBackValue, maximum));
            Volatile.Write(ref upDown, QuantizeAxis(upDownValue, maximum));
            Volatile.Write(ref yaw, QuantizeAxis(yawValue, maximum));
            Interlocked.Exchange(ref latestInputTicks, Stopwatch.GetTimestamp());
        }

        // 데드맨 입력의 활성 상태를 저장한다.
        public void SetDeadman(bool isHeld)
        {
            if (!isHeld)
            {
                Interlocked.Exchange(ref reconnectReleaseRequired, 0);
                ApplyNeutralInput();
                return;
            }

            if (!IsConnected || Volatile.Read(ref reconnectReleaseRequired) != 0)
            {
                Interlocked.Exchange(ref reconnectReleaseRequired, 1);
                ApplyNeutralInput();
                return;
            }

            Volatile.Write(ref deadmanHeld, 1);
        }

        // 이륙 확정 명령을 예약한다.
        [ContextMenu("Control/Takeoff")]
        public void RequestTakeoff()
        {
            if (!IsConnected
                || Volatile.Read(ref reconnectReleaseRequired) != 0)
            {
                return;
            }
            Interlocked.Exchange(ref pendingCommand, (int)DroneControlCommand.Takeoff);
        }

        // 착륙 확정 명령을 예약한다.
        [ContextMenu("Control/Land")]
        public void RequestLand()
        {
            Interlocked.Exchange(ref pendingCommand, (int)DroneControlCommand.Land);
        }

        // 비상 모터 정지 명령을 예약한다.
        [ContextMenu("Control/Emergency")]
        public void RequestEmergency()
        {
            ApplyNeutralInput();
            Interlocked.Exchange(ref pendingCommand, (int)DroneControlCommand.Emergency);
        }

        // 송신 쓰레드에서 최신 제어 상태를 설정된 고정 주기로 전달한다.
        private void SenderLoop()
        {
            IDroneControlTransport initialTransport = ConnectInitialTransport();
            if (initialTransport == null)
            {
                return;
            }

            lock (transportGate)
            {
                transport = initialTransport;
            }

            if (activeMode == DroneControlMode.PythonRelay
                && settings.PreferSharedMemoryWithRelayFallback)
            {
                RequestTemporarySharedMemory();
            }

            long intervalTicks = Math.Max(1, Stopwatch.Frequency / settings.SendRateHz);
            long nextSendTicks = Stopwatch.GetTimestamp();

            while (senderRunning)
            {
                long now = Stopwatch.GetTimestamp();
                if (now < nextSendTicks)
                {
                    long remainingTicks = nextSendTicks - now;
                    int remainingMilliseconds = (int)(
                        remainingTicks * 1000L / Stopwatch.Frequency);
                    if (remainingMilliseconds > 1)
                    {
                        Thread.Sleep(remainingMilliseconds - 1);
                    }
                    else
                    {
                        Thread.Yield();
                    }
                    continue;
                }

                long scheduleJitter = TicksToMicroseconds(Math.Max(0, now - nextSendTicks));
                Interlocked.Exchange(
                    ref lastScheduleJitterMicroseconds,
                    scheduleJitter);
                if (collectTimingSamples)
                {
                    timingWindow.AddJitter(scheduleJitter);
                }
                nextSendTicks += intervalTicks;
                if (nextSendTicks <= now)
                {
                    long skippedSlots = (now - nextSendTicks) / intervalTicks + 1;
                    nextSendTicks += skippedSlots * intervalTicks;
                }

                DroneControlFrame frame = BuildLatestFrame();
                IDroneControlTransport currentTransport;
                lock (transportGate)
                {
                    currentTransport = transport;
                }
                long transportTicks = Stopwatch.GetTimestamp();
                bool sent = false;
                string sendException = null;
                if (currentTransport != null)
                {
                    try
                    {
                        sent = currentTransport.Send(frame, out transportTicks);
                    }
                    catch (Exception exception)
                    {
                        sendException = $"송신 쓰레드 예외: {exception.Message}";
                    }
                }
                if (sent)
                {
                    Interlocked.Increment(ref sentFrames);
                    Volatile.Write(ref consecutiveFailedFrames, 0);
                    long inputTicks = frame.InputTicks > 0 ? frame.InputTicks : now;
                    long inputToSend = TicksToMicroseconds(Math.Max(0, transportTicks - inputTicks));
                    Interlocked.Exchange(
                        ref lastInputToSendMicroseconds,
                        inputToSend);
                    if (collectTimingSamples && frame.InputTicks > 0
                        && Interlocked.Exchange(ref lastMeasuredInputTicks, frame.InputTicks) != frame.InputTicks)
                    {
                        timingWindow.AddInput(inputToSend);
                    }
                    UpdateFlightState(frame.Command);
                }
                else
                {
                    Interlocked.Increment(ref failedFrames);
                    Interlocked.Increment(ref consecutiveFailedFrames);
                    pendingError = sendException
                        ?? currentTransport?.LastError
                        ?? "활성 조종 경로가 없습니다.";
                    if (currentTransport is SharedMemoryControlTransport)
                    {
                        FallbackToStandby(currentTransport);
                    }
                    else if (activeMode == DroneControlMode.PythonRelay)
                    {
                        ReconnectFailedRelay(currentTransport);
                    }
                }
            }

            DroneControlFrame neutral = DroneControlFrame.Neutral(++sequence, Stopwatch.GetTimestamp());
            IDroneControlTransport activeToDispose;
            IDroneControlTransport standbyToDispose;
            lock (transportGate)
            {
                activeToDispose = transport;
                standbyToDispose = standbyTransport;
                transport = null;
                standbyTransport = null;
            }
            activeToDispose?.Send(neutral, out _);
            activeToDispose?.Dispose();
            standbyToDispose?.Dispose();
        }

        // 최신 입력과 확정 명령으로 전송 프레임을 만든다.
        private DroneControlFrame BuildLatestFrame()
        {
            if (Interlocked.Exchange(ref pendingFallbackNeutral, 0) != 0)
            {
                return DroneControlFrame.Neutral(++sequence, Stopwatch.GetTimestamp());
            }

            DroneControlCommand command = (DroneControlCommand)Interlocked.Exchange(
                ref pendingCommand,
                (int)DroneControlCommand.Rc);
            long inputTicks = Interlocked.Read(ref latestInputTicks);
            if (Volatile.Read(ref deadmanHeld) == 0 && command == DroneControlCommand.Rc)
            {
                return DroneControlFrame.Neutral(++sequence, inputTicks);
            }

            return new DroneControlFrame(
                ++sequence,
                inputTicks,
                (short)Volatile.Read(ref leftRight),
                (short)Volatile.Read(ref forwardBack),
                (short)Volatile.Read(ref upDown),
                (short)Volatile.Read(ref yaw),
                command);
        }

        // 선택 모드에 맞는 전송 구현을 생성한다.
        private IDroneControlTransport CreateTransport(DroneControlMode mode)
        {
            return mode switch
            {
                DroneControlMode.UnityDirect => new UnityDirectControlTransport(settings),
                DroneControlMode.SharedMemory => new SharedMemoryControlTransport(settings),
                _ => new PythonRelayControlTransport(settings)
            };
        }

        // Python Relay가 늦게 시작되어도 중립 상태로 연결될 때까지 재시도한다.
        private IDroneControlTransport ConnectInitialTransport()
        {
            bool reportedFailure = false;
            int retryDelayMilliseconds = PythonRelayInitialRetryDelayMilliseconds;
            while (senderRunning)
            {
                IDroneControlTransport candidate = CreateTransport(activeMode);
                if (candidate.Connect())
                {
                    if (!senderRunning)
                    {
                        candidate.Dispose();
                        return null;
                    }

                    ResetControlForReconnect();
                    pendingError = null;
                    return candidate;
                }

                if (!reportedFailure)
                {
                    pendingError = candidate.LastError;
                    reportedFailure = true;
                }
                candidate.Dispose();
                ResetControlForReconnect();

                if (activeMode != DroneControlMode.PythonRelay)
                {
                    senderRunning = false;
                    return null;
                }

                int remaining = retryDelayMilliseconds;
                while (senderRunning && remaining > 0)
                {
                    int delay = Math.Min(ConnectionRetryPollMilliseconds, remaining);
                    Thread.Sleep(delay);
                    remaining -= delay;
                }
                retryDelayMilliseconds = Math.Min(
                    retryDelayMilliseconds * 2,
                    PythonRelayMaximumRetryDelayMilliseconds);
            }

            return null;
        }

        // Relay 송신 실패 시 다른 모듈을 내리지 않고 해당 UDP 세션만 다시 연결한다.
        private void ReconnectFailedRelay(IDroneControlTransport failedTransport)
        {
            lock (transportGate)
            {
                if (!ReferenceEquals(transport, failedTransport)
                    || activeMode != DroneControlMode.PythonRelay)
                {
                    return;
                }
                transport = null;
            }

            failedTransport?.Dispose();
            ResetControlForReconnect();
            IDroneControlTransport replacement = ConnectInitialTransport();
            if (replacement == null)
            {
                return;
            }

            bool accepted = false;
            lock (transportGate)
            {
                if (senderRunning && transport == null)
                {
                    transport = replacement;
                    accepted = true;
                }
            }
            if (!accepted)
            {
                replacement.Dispose();
            }
        }

        // 재연결 전에 이전 조종과 확정 명령을 폐기하고 첫 프레임을 중립으로 고정한다.
        private void ResetControlForReconnect()
        {
            if (Volatile.Read(ref deadmanHeld) != 0)
            {
                Interlocked.Exchange(ref reconnectReleaseRequired, 1);
            }
            ApplyNeutralInput();
            Interlocked.Exchange(ref pendingCommand, (int)DroneControlCommand.Rc);
            Interlocked.Exchange(ref pendingFallbackNeutral, 1);
        }

        // 기존 Relay 송신을 유지한 채 공유메모리 HELLO/ACK를 완료한다.
        private void PrepareSharedMemoryTransport(int generation)
        {
            IDroneControlTransport shared = new SharedMemoryControlTransport(settings);
            if (!shared.Connect())
            {
                pendingError = shared.LastError;
                shared.Dispose();
                if (generation == Volatile.Read(ref transitionGeneration))
                {
                    Interlocked.Exchange(ref temporarySharedRequested, 0);
                    Interlocked.Exchange(ref pendingSharedFallback, 1);
                }
                return;
            }

            lock (transportGate)
            {
                if (generation != Volatile.Read(ref transitionGeneration)
                    || Volatile.Read(ref temporarySharedRequested) == 0
                    || !senderRunning
                    || transport == null
                    || activeMode != DroneControlMode.PythonRelay)
                {
                    shared.Dispose();
                    return;
                }

                standbyTransport = transport;
                standbyMode = activeMode;
                transport = shared;
                activeMode = DroneControlMode.SharedMemory;
                Interlocked.Exchange(ref pendingModeChange, (int)activeMode);
            }
        }

        // 공유메모리 오류 시 이미 연결된 standby Relay를 활성화한다.
        private void FallbackToStandby(IDroneControlTransport failedTransport)
        {
            IDroneControlTransport failedToDispose = null;
            lock (transportGate)
            {
                if (!ReferenceEquals(transport, failedTransport) || standbyTransport == null)
                {
                    return;
                }

                failedToDispose = transport;
                transport = standbyTransport;
                activeMode = standbyMode;
                standbyTransport = null;
                Interlocked.Exchange(ref temporarySharedRequested, 0);
                Interlocked.Increment(ref transitionGeneration);
                Interlocked.Exchange(ref pendingModeChange, (int)activeMode);
                Interlocked.Exchange(ref pendingSharedFallback, 1);
                Interlocked.Exchange(ref pendingFallbackNeutral, 1);
            }
            failedToDispose?.Dispose();
        }

        // 이륙과 착륙 명령의 성공 상태를 반영한다.
        private void UpdateFlightState(DroneControlCommand command)
        {
            if (command == DroneControlCommand.Takeoff)
            {
                Volatile.Write(ref airborne, 1);
            }
            else if (command is DroneControlCommand.Land or DroneControlCommand.Emergency)
            {
                Volatile.Write(ref airborne, 0);
            }
        }

        // 입력을 즉시 중립 상태로 바꾼다.
        private void ApplyNeutralInput()
        {
            Volatile.Write(ref deadmanHeld, 0);
            Volatile.Write(ref leftRight, 0);
            Volatile.Write(ref forwardBack, 0);
            Volatile.Write(ref upDown, 0);
            Volatile.Write(ref yaw, 0);
            Interlocked.Exchange(ref latestInputTicks, Stopwatch.GetTimestamp());
        }

        // 송신 스레드를 안전하게 종료한다.
        private bool StopControl()
        {
            ApplyNeutralInput();
            Interlocked.Exchange(ref temporarySharedRequested, 0);
            Interlocked.Increment(ref transitionGeneration);
            if (!senderRunning && senderThread == null)
            {
                return true;
            }

            senderRunning = false;
            bool stopped = senderThread == null || !senderThread.IsAlive || senderThread.Join(5000);
            if (!stopped)
            {
                Debug.LogError("[Drone Control] 기존 송신 스레드가 종료되지 않아 모드 전환을 중단했습니다.", this);
                return false;
            }

            senderThread = null;
            if (transitionThread != null && transitionThread.IsAlive)
            {
                transitionThread.Join(1000);
            }
            transitionThread = null;
            return true;
        }

        // 정규화 입력을 Tello RC 범위로 변환한다.
        private static int QuantizeAxis(float value, int maximum)
        {
            return Mathf.RoundToInt(Mathf.Clamp(value, -1f, 1f) * maximum);
        }

        // Stopwatch tick을 마이크로초로 변환한다.
        private static long TicksToMicroseconds(long ticks)
        {
            return ticks * 1_000_000L / Stopwatch.Frequency;
        }
    }
}
