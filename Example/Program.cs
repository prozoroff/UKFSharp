using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnscentedKalmanFilter;
using ZedGraph;

namespace Example
{
    class Program
    {
        static void Main(string[] args)
        {
            var filter = new UKF(1, 1);

            var n = 1; //number of state
            var q = 0.05; //std of process 
            var r = 0.3; //std of measurement
            var Q = Matrix.Build.Diagonal(n, n, q * q); //covariance of process
            var R = Matrix.Build.Dense(1, 1, r * r); //covariance of measurement  
            var f = new FEquation(); //nonlinear state equations
            var h = new HEquation(); //measurement equation
            var x = q * Matrix.Build.Random(1, 1);  //s + q * Matrix.Build.Random(1, 1); //initial state with noise
            var P = Matrix.Build.Diagonal(n, n, 1); //initial state covraiance
            var N = 100; //total dynamic steps

            var xV = Matrix.Build.Dense(n, N, 0); //estmate
            var zV = Matrix.Build.Dense(1, N, 0); //measurement

            for (int k = 1; k < N; k++)
            {
                var z = Matrix.Build.Dense(1,1,Math.Sin(k*3.14*5/180)).Add(Matrix.Build.Random(1, 1).Multiply(r)); //measurments
                zV.SetSubMatrix(0, k, z);                                        //save measurment
                var x_and_P = filter.Update(f, x, P, h, z, Q, R);                //ukf 
                x = x_and_P[0];
                P = x_and_P[1];
                xV.SetColumn(k, x.Column(0).ToArray());                          //save estimate
            }

            GraphPane myPane = new GraphPane(new RectangleF(0, 0, 3200, 2400), "Unscented Kalman Filter", "number", "measurement");
            PointPairList list_zV = new PointPairList();
            PointPairList list_xV = new PointPairList();
            for (int i = 0; i < zV.ColumnCount; i++)
            {
                list_zV.Add(i, zV[0, i]);
                list_xV.Add(i, xV[0, i]);
            }
            myPane.AddCurve("measurement", list_zV, Color.Red, SymbolType.Circle);
            myPane.AddCurve("estimate", list_xV, Color.Green, SymbolType.XCross);
            Bitmap bm = new Bitmap(200, 200);
            Graphics g = Graphics.FromImage(bm);
            myPane.AxisChange(g);
            Image im = myPane.Image;
            im.Save("result.png", ImageFormat.Png);
        }
    }

    public class FEquation : IFunction
    {
        public Matrix<double> Process(Matrix<double> x)
        {
            return x;
        }
    }

    public class HEquation : IFunction
    {
        public Matrix<double> Process(Matrix<double> x)
        {
            return x;
        }
    }
}
