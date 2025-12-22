using System.Diagnostics;

namespace dxfilter
{
    static class dt
    {
        private static long lastTick;
        private static double smoothedDt;

        public static double getDT()
        {
            long now = Stopwatch.GetTimestamp();

            if (lastTick == 0)
            {
                lastTick = now;
                return 0.001; // safe default
            }

            double dt = (now - lastTick) / (double)Stopwatch.Frequency;
            lastTick = now;

            dt = Math.Clamp(dt, 0.0002, 0.01);

            smoothedDt = smoothedDt == 0
                ? dt
                : smoothedDt + (dt - smoothedDt) * 0.2;

            return smoothedDt;
        }
    }
}
