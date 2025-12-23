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
        // broke
        [PluginName("d/dx: Central Difference")]
        public class centralDiff : IPositionedPipelineElement<IDeviceReport>
        {
            private Vector2[]? lastPositions = Array.Empty<Vector2>();
            private Vector2 lastPos = Vector2.Zero;
            private int amountPositions;
            private float amountSpacing;
            private bool shouldAvg;
            private bool shouldDt;
            private bool shouldInverse;

            [Property("Reports"), DefaultPropertyValue(5), ToolTip("Default: 5\nRange: 4-1024\n\nNumber of reports to store to calculate derivative from.\nTo calculate how many ms it would smooth over, use (Reports - 1) / RPS.\n\nNote that there will always be 1 report of latency.")]
            public int amountElements
            {
                // clamping to 1024 cuz why not
                set => amountPositions = Math.Clamp(value, 4, 1024);
                get => amountPositions;
            }

            [Property("Spacing between Derivatives"), DefaultPropertyValue(1f), ToolTip("Default: 1\nRange: 0.001-Reports/3\n\nAmount of spacing between reports to calculate derivative.\nA lower value usually tends to give more jitter to your inputs.")]
            public float spacingBD
            {
                set => amountSpacing = Math.Clamp((float)value, (float)0.001, amountPositions / 3);
                get => amountSpacing;
            }

            [Property("Should Average?"), DefaultPropertyValue(true), ToolTip("Should the Derivatives be averaged?")]
            public bool average
            {
                set => shouldAvg = value;
                get => shouldAvg;
            }

            [Property("Should apply deltaTime?"), DefaultPropertyValue(true), ToolTip("Should the output be smoothed with deltaTime?")]
            public bool deltaTime
            {
                set => shouldDt = value;
                get => shouldDt;
            }

            [Property("Should Inverse?"), DefaultPropertyValue(false), ToolTip("Should inverse derivative?")]
            public bool inverse
            {
                set => shouldInverse = value;
                get => shouldInverse;
            }

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    lastPositions = lastPositions.Append(report.Position).ToArray();
                    Vector2 point = math.DerivativeHandle(lastPos, lastPositions, amountPositions, amountSpacing, shouldAvg, shouldDt, shouldInverse);

                    if (lastPositions.Length > amountPositions)
                    {
                        lastPositions = lastPositions.Skip(1).ToArray();
                    }

                    lastPos = report.Position;
                    report.Position = lastPos;
                    device = report;
                }

                Emit?.Invoke(device);
            }
        }

        [PluginName("d/dx: Savitzky-Golay filter")]
        public class savitzkyGolayFilter : IPositionedPipelineElement<IDeviceReport>
        {
            private int windowSize = 7;
            private int polyOrder;
            private int derivativeOrder;
            private double[] sgCoeff;
            // yeah
            private Vector2[] buffer = new Vector2[1024];

            [Property("Window Size"), DefaultPropertyValue(7), ToolTip("Default: 7\nRange: 3-1023\n\nNumber of samples used for smoothing.\n\nLarger values give smoother movement but add a small, fixed delay.")]
            public int WindowSize
            {
                get => windowSize;
                set
                {
                    windowSize = Math.Clamp(value, 3, 1023);
                    RecomputeCoefficients();
                }
            }

            [Property("Polynomial Order"), DefaultPropertyValue(2), ToolTip("Default: 2\nRange: 1 - (WindowSize - 2)\n\nControls how well sharp corners and curves are preserved.\n\nHigher values preserve detail but reduce smoothing.")]
            public int PolyOrder
            {
                get => polyOrder;
                set
                {
                    polyOrder = Math.Clamp(value, 1, windowSize - 2);
                    RecomputeCoefficients();
                }
            }

            [Property("Derivative Order"), DefaultPropertyValue(0), ToolTip("Default: 0\nRange: 1 - PolynomalOrder\n\nDerivative order for Savitzky-Golay filter.\n\n0 = smooth position (most common)\n1 = smooth velocity (advanced)\n2 = smooth acceleration (rarely used)")]
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
                var AT = math.Transpose(A);
                var ATA = math.Multiply(AT, A);
                var ATAinv = math.Invert(ATA);
                var G = math.Multiply(ATAinv, AT);

                double factorial = math.Factorial(derivative);
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

        [PluginName("d/dx: Secant Slope")]
        public class secantSlope : IPositionedPipelineElement<IDeviceReport>
        {
            private Vector2 lastPos = Vector2.Zero;
            private int reportsDifference;
            private int currentReport;

            [Property("Reports"), DefaultPropertyValue(1), ToolTip("Default: 1\nRange: 1-2147483647\n\nHow many reports should be skipped until calculating the Secant slope?\n\nEffectively divides your RPS by that many reports. However, can resemble\na kind of smoothing/snappiness in lower report values.")]
            public int amountReports
            {
                // clamping to 1024 cuz why not
                set => reportsDifference = Math.Clamp(value, 1, int.MaxValue);
                get => reportsDifference;
            }

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    if (lastPos != null)
                    {
                        if (currentReport % reportsDifference == 0)
                        {
                            report.Position = math.SecantSlope(lastPos, report.Position, reportsDifference);
                            lastPos = report.Position;
                        } else
                        {
                            report.Position = lastPos;
                        }

                        currentReport++;
                    } else
                    {
                        lastPos = report.Position;
                    }

                    device = report;
                }

                Emit?.Invoke(device);
            }
        }

    }
}