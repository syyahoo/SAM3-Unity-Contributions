using System;
using System.Buffers.Binary;

namespace StrikerPC.Control
{
    public static class ControlPacketCodec
    {
        public const uint Magic = 0x31524354;
        public const ushort Version = 1;
        public const int PacketSize = 40;

        // Python struct 규격과 동일한 little-endian 패킷을 만든다.
        public static void Encode(Span<byte> destination, in DroneControlFrame frame)
        {
            if (destination.Length < PacketSize)
            {
                throw new ArgumentException($"조종 패킷 버퍼는 {PacketSize}바이트 이상이어야 합니다.", nameof(destination));
            }

            destination.Slice(0, PacketSize).Clear();
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(0, 4), Magic);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(4, 2), Version);
            BinaryPrimitives.WriteUInt64LittleEndian(destination.Slice(8, 8), frame.Sequence);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(16, 8), frame.InputTicks);
            BinaryPrimitives.WriteInt16LittleEndian(destination.Slice(24, 2), frame.LeftRight);
            BinaryPrimitives.WriteInt16LittleEndian(destination.Slice(26, 2), frame.ForwardBack);
            BinaryPrimitives.WriteInt16LittleEndian(destination.Slice(28, 2), frame.UpDown);
            BinaryPrimitives.WriteInt16LittleEndian(destination.Slice(30, 2), frame.Yaw);
            destination[32] = (byte)frame.Command;
        }
    }
}
