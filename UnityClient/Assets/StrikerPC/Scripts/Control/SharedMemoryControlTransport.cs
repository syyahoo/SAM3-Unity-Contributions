using System;
using System.Buffers.Binary;
using System.Diagnostics;
using System.IO.MemoryMappedFiles;
using System.Security.Cryptography;
using System.Threading;

namespace StrikerPC.Control
{
    public sealed class SharedMemoryControlTransport : IDroneControlTransport
    {
        public const uint MemoryMagic = 0x31435054;
        public const ushort MemoryVersion = 1;
        public const int WriterSlotSize = 48;
        public const int ReaderSlotSize = 48;
        public const int MemorySize = WriterSlotSize + ReaderSlotSize;

        private const ushort ActiveFlag = 1;
        private const int AckSuccess = 1;
        private const uint ReaderReady = 1;
        private const int SequenceOffset = 16;
        private const int AckSessionOffset = 48;
        private const int AckSequenceOffset = 56;
        private const int AckStatusOffset = 72;
        private const int ReaderStateOffset = 76;
        private const int ReaderHeartbeatOffset = 80;

        private readonly TelloControlSettings settings;
        private readonly object writeLock = new object();
        private readonly byte[] writerSlot = new byte[WriterSlotSize];
        private MemoryMappedFile mappedFile;
        private MemoryMappedViewAccessor accessor;
        private ulong sessionId;
        private long wireSequence;
        private long lastReaderHeartbeat;
        private long lastHeartbeatChangeTicks;

        public bool IsConnected { get; private set; }
        public string LastError { get; private set; } = string.Empty;

        // 공유메모리 설정을 보관한다.
        public SharedMemoryControlTransport(TelloControlSettings settings)
        {
            this.settings = settings;
        }

        // 명명 메모리를 열고 Pilot Reader와 HELLO/ACK를 교환한다.
        public bool Connect()
        {
            try
            {
                string memoryName = string.IsNullOrWhiteSpace(settings.SharedControlMemoryName)
                    ? "TelloPilotControl_v1"
                    : settings.SharedControlMemoryName;
                mappedFile = MemoryMappedFile.CreateOrOpen(
                    memoryName,
                    MemorySize,
                    MemoryMappedFileAccess.ReadWrite);
                accessor = mappedFile.CreateViewAccessor(
                    0,
                    MemorySize,
                    MemoryMappedFileAccess.ReadWrite);
                sessionId = CreateSessionId();
                wireSequence = 0;
                lastReaderHeartbeat = 0;
                lastHeartbeatChangeTicks = Stopwatch.GetTimestamp();

                long helloSequence = WriteFrame(
                    DroneControlFrame.Neutral(0, Stopwatch.GetTimestamp()),
                    DroneControlCommand.Hello,
                    true);
                if (!WaitForAck(helloSequence, settings.SharedHandshakeTimeoutSeconds))
                {
                    LastError = "Pilot 공유메모리 제어 Reader가 응답하지 않습니다.";
                    Dispose();
                    return false;
                }

                IsConnected = true;
                return true;
            }
            catch (Exception exception)
            {
                LastError = exception.Message;
                Dispose();
                return false;
            }
        }

        // 최신 조종 프레임을 Writer 슬롯에 기록한다.
        public bool Send(in DroneControlFrame frame, out long transportTicks)
        {
            transportTicks = Stopwatch.GetTimestamp();
            if (!IsConnected || accessor == null)
            {
                return false;
            }

            try
            {
                if (!HasFreshReaderHeartbeat())
                {
                    LastError = "Pilot 공유메모리 제어 heartbeat가 만료되었습니다.";
                    return false;
                }

                long sequence = WriteFrame(frame, frame.Command, true);
                transportTicks = Stopwatch.GetTimestamp();
                if (frame.Command is DroneControlCommand.Takeoff
                    or DroneControlCommand.Land
                    or DroneControlCommand.Emergency)
                {
                    if (!WaitForAck(sequence, settings.SharedCommandTimeoutSeconds))
                    {
                        LastError = "Pilot이 공유메모리 명령을 확인하지 않았습니다.";
                        return false;
                    }
                }
                return true;
            }
            catch (Exception exception)
            {
                LastError = exception.Message;
                return false;
            }
        }

        // inactive 중립 프레임을 남기고 공유메모리 핸들을 닫는다.
        public void Dispose()
        {
            if (accessor != null && sessionId != 0)
            {
                try
                {
                    WriteFrame(
                        DroneControlFrame.Neutral(0, Stopwatch.GetTimestamp()),
                        DroneControlCommand.Neutral,
                        false);
                }
                catch (Exception)
                {
                }
            }

            IsConnected = false;
            accessor?.Dispose();
            mappedFile?.Dispose();
            accessor = null;
            mappedFile = null;
            sessionId = 0;
        }

        // 음수 sequence 기록 후 전체 슬롯을 양수 sequence로 확정한다.
        private long WriteFrame(
            in DroneControlFrame frame,
            DroneControlCommand command,
            bool active)
        {
            lock (writeLock)
            {
                long sequence = ++wireSequence;
                EncodeWriterSlot(frame, command, active, -sequence);
                accessor.WriteArray(0, writerSlot, 0, writerSlot.Length);
                BinaryPrimitives.WriteInt64LittleEndian(
                    writerSlot.AsSpan(SequenceOffset, sizeof(long)),
                    sequence);
                accessor.WriteArray(0, writerSlot, 0, writerSlot.Length);
                accessor.Flush();
                return sequence;
            }
        }

        // 고정 48바이트 Writer 슬롯을 little-endian으로 만든다.
        private void EncodeWriterSlot(
            in DroneControlFrame frame,
            DroneControlCommand command,
            bool active,
            long sequence)
        {
            Array.Clear(writerSlot, 0, writerSlot.Length);
            Span<byte> span = writerSlot;
            BinaryPrimitives.WriteUInt32LittleEndian(span.Slice(0, 4), MemoryMagic);
            BinaryPrimitives.WriteUInt16LittleEndian(span.Slice(4, 2), MemoryVersion);
            BinaryPrimitives.WriteUInt16LittleEndian(span.Slice(6, 2), active ? ActiveFlag : (ushort)0);
            BinaryPrimitives.WriteUInt64LittleEndian(span.Slice(8, 8), sessionId);
            BinaryPrimitives.WriteInt64LittleEndian(span.Slice(SequenceOffset, 8), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(span.Slice(24, 8), frame.InputTicks);
            BinaryPrimitives.WriteInt16LittleEndian(span.Slice(32, 2), frame.LeftRight);
            BinaryPrimitives.WriteInt16LittleEndian(span.Slice(34, 2), frame.ForwardBack);
            BinaryPrimitives.WriteInt16LittleEndian(span.Slice(36, 2), frame.UpDown);
            BinaryPrimitives.WriteInt16LittleEndian(span.Slice(38, 2), frame.Yaw);
            span[40] = (byte)command;
        }

        // 같은 session과 sequence의 안정된 Reader ACK를 기다린다.
        private bool WaitForAck(long sequence, float timeoutSeconds)
        {
            long deadline = Stopwatch.GetTimestamp()
                + (long)(Math.Max(0.1f, timeoutSeconds) * Stopwatch.Frequency);
            while (Stopwatch.GetTimestamp() < deadline)
            {
                if (TryReadAck(sequence, out int status))
                {
                    return status == AckSuccess;
                }
                Thread.Sleep(1);
            }
            return false;
        }

        // Reader 슬롯을 sequence 전후 비교로 원자적으로 읽는다.
        private bool TryReadAck(long expectedSequence, out int status)
        {
            status = 0;
            if (accessor == null)
            {
                return false;
            }

            long firstSequence = accessor.ReadInt64(AckSequenceOffset);
            if (firstSequence <= 0)
            {
                return false;
            }
            ulong ackSession = accessor.ReadUInt64(AckSessionOffset);
            int ackStatus = accessor.ReadInt32(AckStatusOffset);
            uint readerState = accessor.ReadUInt32(ReaderStateOffset);
            long heartbeat = accessor.ReadInt64(ReaderHeartbeatOffset);
            if (accessor.ReadInt64(AckSequenceOffset) != firstSequence)
            {
                return false;
            }

            ObserveHeartbeat(heartbeat);
            if (readerState != ReaderReady
                || ackSession != sessionId
                || firstSequence != expectedSequence)
            {
                return false;
            }
            status = ackStatus;
            return true;
        }

        // Reader heartbeat 변화가 제한 시간 안에 있었는지 확인한다.
        private bool HasFreshReaderHeartbeat()
        {
            long heartbeat = accessor.ReadInt64(ReaderHeartbeatOffset);
            ObserveHeartbeat(heartbeat);
            long elapsed = Stopwatch.GetTimestamp() - lastHeartbeatChangeTicks;
            return heartbeat != 0
                && elapsed <= settings.SharedHeartbeatTimeoutSeconds * Stopwatch.Frequency;
        }

        // heartbeat가 바뀐 시각을 Unity 성능 카운터로 기록한다.
        private void ObserveHeartbeat(long heartbeat)
        {
            if (heartbeat == 0 || heartbeat == lastReaderHeartbeat)
            {
                return;
            }
            lastReaderHeartbeat = heartbeat;
            lastHeartbeatChangeTicks = Stopwatch.GetTimestamp();
        }

        // 충돌 가능성이 낮은 0이 아닌 session ID를 만든다.
        private static ulong CreateSessionId()
        {
            Span<byte> bytes = stackalloc byte[sizeof(ulong)];
            RandomNumberGenerator.Fill(bytes);
            ulong value = BinaryPrimitives.ReadUInt64LittleEndian(bytes);
            return value == 0 ? 1UL : value;
        }
    }
}
