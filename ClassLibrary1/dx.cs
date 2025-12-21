using System;
using System.Linq;
using System.Numerics;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using MathNet.Numerics;
using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra.Complex32;
using System.Runtime.InteropServices;
using System.Diagnostics;
using OpenTabletDriver.Plugin;
using System.Drawing;

namespace dxfilter
{
    public class dx
    {
        [PluginName("d/dx smoothing")]
        public class derivativeSmooth : IPositionedPipelineElement<IDeviceReport> {
        
            private Vector2[]? lastPositions = { };
            private int amountPositions;
            private float amountSpacing;

            [Property("Amount of Elements"), DefaultPropertyValue(5), ToolTip ("Default: 2\n\n" + "idk what this does LOOOL")]
            public int amountElements
            {
                // clamping to 1024 cuz why not
                set => amountPositions = Math.Clamp(value, 4, 1024);
                get => amountPositions;
            }

            [Property("Spacing between Derivatives"), DefaultPropertyValue(1f), ToolTip("Default: 1.0\n\n" + "idk what this does LOOOL")]
            public float spacingBD
            {
                // clamping to 1024 cuz why not
                set => amountSpacing = Math.Clamp(value, 0, amountPositions/2);
                get => amountSpacing;
            }

            public event Action<IDeviceReport>? Emit;

            public PipelinePosition Position => PipelinePosition.PreTransform;

            public void Consume(IDeviceReport device)
            {
                if (device is ITabletReport report)
                {
                    Vector2 point = DerivativeFunc(report.Position, lastPositions, amountPositions, amountSpacing);
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
                    Log.Debug("d/dx", "ARRAY NULL! NOT CALC..");
                    return lastInput;
                } else
                {
                    if (array.Length < amountElements)
                    {
                        Log.Debug("d/dx", "ARRAY TOO SHORT! NOT CALC.");
                        array.Append(lastInput);
                        return lastInput;
                    } else
                    {
                        
                        float[] yPos = { };
                        float[] xPos = { };

                        for (int i = 0;  i < array.Length; i++)
                        {
                            yPos.Append(array[i].Y);
                            xPos.Append(array[i].X);
                        }

                        float[] derivatedArrayX = CentralDifference(xPos, amountSpacing);
                        float[] derivatedArrayY = CentralDifference(yPos, amountSpacing);

                        Log.Debug("d/dx", "d/dx: " + derivatedArrayX);
                        Log.Debug("d/dx", "d/dy: " + derivatedArrayY);

                        float finalX = derivatedArrayX.Average();
                        float finalY = derivatedArrayY.Average();

                        // i deadass dont know what im doing
                        Vector2 FinalVector = new Vector2(lastInput.X + finalX, lastInput.Y + finalY);

                        array = array.Skip(1).ToArray();
                        array.Append(lastInput);

                        return FinalVector;
                    }
                }
            }
        
            
        }
    }
}
