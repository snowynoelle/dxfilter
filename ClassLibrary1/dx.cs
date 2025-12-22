using System.Numerics;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin;
using System.Diagnostics;

namespace dxfilter
{
    public class dx
    {
        // [PluginName("d/dx smoothing")]

        [PluginName("d/dx: Savitzky-Golay filter")]
        public class savitzkyGolayFilter : IPositionedPipelineElement<IDeviceReport>
        {
            private int windowSize = 7;
            private int polyOrder;
            private int derivativeOrder;
            private double[] sgCoeff;
            // yeah
            private Vector2[] buffer = new Vector2[1024];

            [Property("Window Size"), DefaultPropertyValue(7), ToolTip("Number of samples used for smoothing.\n\nLarger values give smoother movement but add a small, fixed delay.\n\nRecommended: 7")]
            public int WindowSize
            {
                get => windowSize;
                set
                {
                    windowSize = Math.Clamp(value, 3, 1023);
                    RecomputeCoefficients();
                }
            }

            [Property("Polynomial Order"), DefaultPropertyValue(2), ToolTip("Controls how well sharp corners and curves are preserved.\n\nHigher values preserve detail but reduce smoothing.\n\nRecommended: 2")]
            public int PolyOrder
            {
                get => polyOrder;
                set
                {
                    polyOrder = Math.Clamp(value, 1, windowSize - 2);
                    RecomputeCoefficients();
                }
            }

            [Property("Derivative Order"), DefaultPropertyValue(0), ToolTip("Derivative order for Savitzky-Golay filter.\n\n0 = smooth position (most common)\n1 = smooth velocity (advanced)\n2 = smooth acceleration (rarely used)\n\n\"Must be ≤ Polynomial Order")]
            public int DerivativeOrder
            {
                get => derivativeOrder;
                set
                {
                    derivativeOrder = Math.Clamp(value, 0, polyOrder);
                    RecomputeCoefficients();
                }
            }

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            private int index = 0;
            private int count = 0;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    buffer[index] = report.Position;
                    index = (index + 1) % windowSize;
                    count = Math.Min(count + 1, windowSize);
                    float dt = (float)dxfilter.dt.getDT();

                    if (count == windowSize)
                    {
                        if (derivativeOrder == 0)
                        {
                            Vector2 smoothPos = ApplySG(buffer, index);
                            report.Position = smoothPos;
                        }
                        else
                        {
                            Vector2 velocity = ApplySG(buffer, index) / dt;
                            report.Position += velocity * dt;
                        }
                    }
                }

                Emit?.Invoke(device);
            }

            static double[,] BuildVandermonde(int window, int order)
            {
                int m = (window - 1) / 2;
                double[,] A = new double[window, order + 1];

                for (int i = 0; i < window; i++)
                {
                    double t = i - m;
                    double value = 1.0;

                    for (int j = 0; j <= order; j++)
                    {
                        A[i, j] = value;
                        value *= t;
                    }
                }

                return A;
            }

            void RecomputeCoefficients()
            {
                sgCoeff = ComputeSGCoefficients(
                    windowSize,
                    polyOrder,
                    derivativeOrder
                );
            }

            static double[] ComputeSGCoefficients(int window, int order, int derivative)
            {
                var A = BuildVandermonde(window, order);
                var AT = dxfilter.math.Transpose(A);
                var ATA = dxfilter.math.Multiply(AT, A);
                var ATAinv = dxfilter.math.Invert(ATA);
                var G = dxfilter.math.Multiply(ATAinv, AT);

                double factorial = dxfilter.math.Factorial(derivative);
                double[] coeff = new double[window];

                for (int i = 0; i < window; i++)
                    coeff[i] = G[derivative, i] * factorial;

                return coeff;
            }

            Vector2 ApplySG(Vector2[] buffer, int head)
            {
                Vector2 sum = Vector2.Zero;

                for (int i = 0; i < sgCoeff.Length; i++)
                {
                    int idx = (head + i) % sgCoeff.Length;
                    sum += buffer[idx] * (float)sgCoeff[i];
                }

                return sum;
            }
        }

        [PluginName("d/dx: Central Difference")]
        public class centralDiffAvg : IPositionedPipelineElement<IDeviceReport>
        {
            private Vector2[]? lastPositions = { };
            private int amountPositions;
            private float amountSpacing;
            private bool shouldAvg;
            private bool shouldDt;

            [Property("Reports"), DefaultPropertyValue(5), ToolTip("Default: 5\n\n" + "Number of reports to store to calculate derivative from.\n" + "To calculate how many ms it would smooth over, use (Reports - 1) / RPS.\n\n" + "Note that there will always be 1 report of latency.")]
            public int amountElements
            {
                // clamping to 1024 cuz why not
                set => amountPositions = Math.Clamp(value, 4, 1024);
                get => amountPositions;
            }

            [Property("Spacing between Derivatives"), DefaultPropertyValue(1f), ToolTip("Default: 1\n\n" + "Amount of spacing between reports to calculate derivative.\nA lower value usually tends to give more jitter to your inputs. ")]
            public float spacingBD
            {
                // clamping to 1024 cuz why not
                set => amountSpacing = Math.Clamp(value, 0, amountPositions / 3);
                get => amountSpacing;
            }

            [Property("Should aAverage?"), DefaultPropertyValue(true), ToolTip("Should the Derivatives be averaged?")]
            public bool averagePref
            {
                // clamping to 1024 cuz why not
                set => shouldAvg = value;
                get => shouldAvg;
            }

            [Property("Should apply deltaTime?"), DefaultPropertyValue(true), ToolTip("Should the output be smoothed with deltaTime?")]
            public bool dtPref
            {
                // clamping to 1024 cuz why not
                set => shouldDt = value;
                get => shouldDt;
            }

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    lastPositions = lastPositions.Append(report.Position).ToArray();
                    Vector2 point = dxfilter.math.DerivativeHandle(report.Position, lastPositions, amountPositions, amountSpacing, shouldAvg, dtPref);
                    
                    if (lastPositions.Length > amountPositions)
                    {
                        lastPositions = lastPositions.Skip(1).ToArray();
                    }

                    report.Position = point;
                    device = report;
                }

                Emit?.Invoke(device);
            }
        }
    }
}