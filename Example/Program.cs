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
            var filter = new UKF();

            List<double> measurements = new List<double>();
            List<double> states = new List<double>();
     
            Random rnd = new Random();

            for (int k = 0; k < 100; k++)
            {
                var measurement = Math.Sin(k * 3.14 * 5 / 180) + (double)rnd.Next(50) / 100;
                measurements.Add(measurement);
                filter.Update(new[] { measurement });
                states.Add(filter.getState()[0]);
            }

            GraphPane myPane = new GraphPane(new RectangleF(0, 0, 3200, 2400), "Unscented Kalman Filter", "number", "measurement");
            PointPairList measurementsPairs = new PointPairList();
            PointPairList statesPairs = new PointPairList();
            for (int i = 0; i < measurements.Count; i++)
            {
                measurementsPairs.Add(i, measurements[i]);
                statesPairs.Add(i, states[i]);
            }
            myPane.AddCurve("measurement", measurementsPairs, Color.Red, SymbolType.Circle);
            myPane.AddCurve("estimate", statesPairs, Color.Green, SymbolType.XCross);
            Bitmap bm = new Bitmap(200, 200);
            Graphics g = Graphics.FromImage(bm);
            myPane.AxisChange(g);
            Image im = myPane.Image;
            im.Save("result.png", ImageFormat.Png);
        }
    }
}
