using System;
using System.Collections.ObjectModel;
using UnscentedKalmanFilter;

namespace DemoApp
{
    class MainWindowViewModel : INPCBase
    {

        public ObservableCollection<Measurement> Measurements { get; set; }
        public ObservableCollection<Measurement> Estimates { get; set; }


        public MainWindowViewModel()
        {
            Measurements = new ObservableCollection<Measurement>();
            Estimates = new ObservableCollection<Measurement>();

            var filter = new UKF();
            var N = 100;

            for (int k = 1; k < N; k++)
            {
                double[] z = ProcessBuilder.SineWave(k);
                filter.Update(z);
                var state = filter.getState();
                var covariance = filter.getCovariance();

                Measurements.Add(new Measurement() { Value = z[0], Time = TimeSpan.FromSeconds(k) });
                Estimates.Add(new Measurement() { Value = state[0], Time = TimeSpan.FromSeconds(k), Variance = covariance[0, 0] });
            }
        }
    }

    public static class ProcessBuilder
    {
        private static Random rnd = new Random();

        public static double[] SineWave(int iteration)
        {
            return new[] { Math.Sin(iteration * 3.14 * 5 / 180) + (double)rnd.Next(50) / 100 };
        }
    }

    public struct Measurement
    {
        private double variance;
        public double Value { get; set; }
        public TimeSpan Time { get; set; }
        public double Variance {
            get
            {
                return  variance;
            }
            set
            {
                variance = value;
                UpperDeviation = Value + Math.Sqrt(variance);
                LowerDeviation = Value - Math.Sqrt(variance);
            }
        }
        public double UpperDeviation { get;private set; }
        public double LowerDeviation{ get; private set; }
    }
}




