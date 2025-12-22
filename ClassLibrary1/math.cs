using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace dxfilter
{
    static class math
    {
        public static double[,] Transpose(double[,] A)
        {
            int rows = A.GetLength(0);
            int cols = A.GetLength(1);

            var T = new double[cols, rows];

            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    T[j, i] = A[i, j];

            return T;
        }

        public static double[,] Multiply(double[,] A, double[,] B)
        {
            int aRows = A.GetLength(0);
            int aCols = A.GetLength(1);
            int bCols = B.GetLength(1);

            var C = new double[aRows, bCols];

            for (int i = 0; i < aRows; i++)
                for (int k = 0; k < aCols; k++)
                    for (int j = 0; j < bCols; j++)
                        C[i, j] += A[i, k] * B[k, j];

            return C;
        }

        public static double[,] Identity(int n)
        {
            var I = new double[n, n];
            for (int i = 0; i < n; i++)
                I[i, i] = 1.0;
            return I;
        }

        public static double[,] Invert(double[,] A)
        {
            int n = A.GetLength(0);
            var aug = new double[n, 2 * n];

            // build augmented matrix [A | I]
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    aug[i, j] = A[i, j];

            for (int i = 0; i < n; i++)
                aug[i, i + n] = 1.0;

            // Gauss–Jordan elimination
            for (int col = 0; col < n; col++)
            {
                // pivot
                int pivot = col;
                double max = Math.Abs(aug[col, col]);

                for (int row = col + 1; row < n; row++)
                {
                    double val = Math.Abs(aug[row, col]);
                    if (val > max)
                    {
                        max = val;
                        pivot = row;
                    }
                }

                if (max < 1e-12)
                    throw new InvalidOperationException("Matrix is singular");

                // swap rows
                if (pivot != col)
                    for (int j = 0; j < 2 * n; j++)
                        (aug[col, j], aug[pivot, j]) = (aug[pivot, j], aug[col, j]);

                // normalize row
                double div = aug[col, col];
                for (int j = 0; j < 2 * n; j++)
                    aug[col, j] /= div;

                // eliminate other rows
                for (int row = 0; row < n; row++)
                {
                    if (row == col) continue;
                    double factor = aug[row, col];
                    for (int j = 0; j < 2 * n; j++)
                        aug[row, j] -= factor * aug[col, j];
                }
            }

            // extract inverse
            var inv = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    inv[i, j] = aug[i, j + n];

            return inv;
        }

        public static double Factorial(int n)
        {
            double result = 1.0;
            for (int i = 2; i <= n; i++)
                result *= i;
            return result;
        }

        public static float[] CentralDifference(float[] points, float spacing)
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

        public static Vector2 DerivativeHandle(Vector2 lastInput, Vector2[]? array, int amountElements, float amountSpacing, bool shouldAverage, bool shouldDeltaTime)
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
                    float finalX;
                    float finalY;

                    if (shouldAverage == true)
                    {
                        finalX = derivatedArrayX.Average();
                        finalY = derivatedArrayY.Average();
                    } else
                    {
                        finalX = derivatedArrayX.Last();
                        finalY = derivatedArrayY.Last();
                    }
                    
                    Vector2 FinalVector = new Vector2(lastInput.X + finalX, lastInput.Y + finalY);

                    if (shouldDeltaTime == true)
                    {
                        Vector2 vel = new Vector2(finalX, finalY);
                        Vector2 temp = FinalVector;
                        FinalVector = temp + vel * (float)dt.getDT();
                       
                    }

                    return FinalVector;
                }
            }
        }
    }
}
