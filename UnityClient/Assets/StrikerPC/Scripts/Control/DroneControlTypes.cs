using System;

namespace StrikerPC.Control
{
    public enum DroneControlMode
    {
        UnityDirect = 0,
        PythonRelay = 1,
        SharedMemory = 2
    }

    public enum DroneControlCommand : byte
    {
        Rc = 0,
        Neutral = 1,
        Takeoff = 2,
        Land = 3,
        Emergency = 4,
        Hello = 5
    }

    [Serializable]
    public readonly struct DroneControlFrame
    {
        public readonly ulong Sequence;
        public readonly long InputTicks;
        public readonly short LeftRight;
        public readonly short ForwardBack;
        public readonly short UpDown;
        public readonly short Yaw;
        public readonly DroneControlCommand Command;

        // 전송할 조종 상태를 고정값으로 만든다.
        public DroneControlFrame(
            ulong sequence,
            long inputTicks,
            short leftRight,
            short forwardBack,
            short upDown,
            short yaw,
            DroneControlCommand command)
        {
            Sequence = sequence;
            InputTicks = inputTicks;
            LeftRight = leftRight;
            ForwardBack = forwardBack;
            UpDown = upDown;
            Yaw = yaw;
            Command = command;
        }

        // 중립 조종 프레임을 생성한다.
        public static DroneControlFrame Neutral(ulong sequence, long inputTicks)
        {
            return new DroneControlFrame(sequence, inputTicks, 0, 0, 0, 0, DroneControlCommand.Neutral);
        }
    }

    public interface IDroneControlTransport : IDisposable
    {
        bool IsConnected { get; }
        string LastError { get; }
        bool Connect();
        bool Send(in DroneControlFrame frame, out long transportTicks);
    }
}
