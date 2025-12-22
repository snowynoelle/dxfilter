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
                var AT = dxfilter.matrix.Transpose(A);
                var ATA = dxfilter.matrix.Multiply(AT, A);
                var ATAinv = dxfilter.matrix.Invert(ATA);
                var G = dxfilter.matrix.Multiply(ATAinv, AT);

                double factorial = dxfilter.matrix.Factorial(derivative);
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

        [PluginName("d/dx: Central Difference Average")]
        public class centralDiffAvg : IPositionedPipelineElement<IDeviceReport>
        {
            private Vector2[]? lastPositions = { };
            private int amountPositions;
            private float amountSpacing;

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

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    Vector2 point = DerivativeFunc(report.Position, lastPositions, amountPositions, amountSpacing);
                    lastPositions = lastPositions.Append(report.Position).ToArray();

                    if (lastPositions.Length > amountPositions)
                    {
                        lastPositions = lastPositions.Skip(1).ToArray();
                    }

                    report.Position = point;
                    device = report;
                }

                Emit?.Invoke(device);
            }

            private static float[] CentralDifference(float[] points, float spacing)
            {
                int n = points.Length;
                if (n < 3) return new float[0];

                float[] derivative = new float[n - 2];

                for (int i = 1; i < n - 1; i++)
                {
                    derivative[i - 1] = (points[i + 1] - points[i - 1]) / (2 * spacing);
                }

                return derivative;
            }

            private static Vector2 DerivativeFunc(Vector2 lastInput, Vector2[]? array, int amountElements, float amountSpacing)
            {
                if (array == null)
                {
                    return lastInput;
                }
                else
                {
                    if (array.Length < amountElements)
                    {
                        return lastInput;
                    }
                    else
                    {

                        float[] yPos = { };
                        float[] xPos = { };

                        for (int i = 0; i < array.Length; i++)
                        {
                            yPos = yPos.Append(array[i].Y).ToArray();
                            xPos = xPos.Append(array[i].X).ToArray();
                        }

                        float[] derivatedArrayX = CentralDifference(xPos, amountSpacing);
                        float[] derivatedArrayY = CentralDifference(yPos, amountSpacing);

                        float finalX = derivatedArrayX.Average();
                        float finalY = derivatedArrayY.Average();

                        // i deadass dont know what im doing
                        Vector2 FinalVector = new Vector2(lastInput.X + finalX, lastInput.Y + finalY);

                        return FinalVector;
                    }
                }
            }
        }

        [PluginName("d/dx: Central Difference")]
        public class centralDiffInst : IPositionedPipelineElement<IDeviceReport>
        {
            private Vector2[]? lastPositions = { };
            private int amountPositions = 3;
            private float amountSpacing = 1;

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    Vector2 point = DerivativeFunc(report.Position, lastPositions, amountPositions, amountSpacing);
                    lastPositions = lastPositions.Append(report.Position).ToArray();

                    if (lastPositions.Length > amountPositions)
                    {
                        lastPositions = lastPositions.Skip(1).ToArray();
                    }

                    report.Position = point;
                    device = report;
                }

                Emit?.Invoke(device);
            }

            private static float[] CentralDifference(float[] points, float spacing)
            {
                int n = points.Length;
                if (n < 3) return new float[0];

                float[] derivative = new float[n - 2];

                for (int i = 1; i < n - 1; i++)
                {
                    derivative[i - 1] = (points[i + 1] - points[i - 1]) / (2 * spacing);
                }

                return derivative;
            }

            private static Vector2 DerivativeFunc(Vector2 lastInput, Vector2[]? array, int amountElements, float amountSpacing)
            {
                if (array == null)
                {
                    return lastInput;
                }
                else
                {
                    if (array.Length < amountElements)
                    {
                        return lastInput;
                    }
                    else
                    {

                        float[] yPos = { };
                        float[] xPos = { };

                        for (int i = 0; i < array.Length; i++)
                        {
                            yPos = yPos.Append(array[i].Y).ToArray();
                            xPos = xPos.Append(array[i].X).ToArray();
                        }

                        float[] derivatedArrayX = CentralDifference(xPos, amountSpacing);
                        float[] derivatedArrayY = CentralDifference(yPos, amountSpacing);

                        float finalX = derivatedArrayX.Last();
                        float finalY = derivatedArrayY.Last();

                        // i deadass dont know what im doing
                        Vector2 FinalVector = new Vector2(lastInput.X + finalX, lastInput.Y + finalY);

                        return FinalVector;
                    }
                }
            }
        }
    }
}