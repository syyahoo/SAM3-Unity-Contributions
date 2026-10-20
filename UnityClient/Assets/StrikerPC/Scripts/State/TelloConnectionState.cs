using System;

namespace StrikerPC.State
{
    [Serializable]
    public sealed class TelloConnectionState
    {
        public bool sdkConnected;
        public bool stateConnected;
        public bool videoConnected;
        public bool isFlying;
        public int wifiSignalPercent = -1;
        public int batteryPercent;
        public int heightCentimeters;
        public int flightTimeSeconds;
        public float pitch;
        public float roll;
        public float yaw;
        public float velocityXCentimetersPerSecond;
        public float velocityYCentimetersPerSecond;
        public float velocityZCentimetersPerSecond;
        public long receivedUtcMilliseconds;

        public float SpeedMetersPerSecond => (float)Math.Sqrt(
            velocityXCentimetersPerSecond * velocityXCentimetersPerSecond
            + velocityYCentimetersPerSecond * velocityYCentimetersPerSecond
            + velocityZCentimetersPerSecond * velocityZCentimetersPerSecond) / 100f;
    }
}
