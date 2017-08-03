using System;
using System.IO;
using System.Linq;
using System.Windows;
using System.Collections.ObjectModel;
using UnscentedKalmanFilter;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

namespace DemoApp
{
    class MainWindowViewModel : INPCBase
    {

        public ObservableCollection<Measurement> Measurements { get; set; }
        public ObservableCollection<Measurement> Estimates { get; set; }



        public Matrix<double> Q { get; set; }
        public Matrix<double> R { get; set; }
        public FEquation f { get; set; }
        public HEquation h { get; set; }
        public Matrix<double> x { get; set; }
        public Matrix<double> P { get; set; }


        private int n; //number of state
        private double q; //std of process 
        private double r; //std of measurement
        private int N;//total dynamic steps

        public MainWindowViewModel()
        {
            Measurements = new ObservableCollection<Measurement>();
            Estimates = new ObservableCollection<Measurement>();
        }


        public void Run()
        {
            var filter = new UKF(1, 1);

            n = 1;
            q = 0.05;
            r = 0.3;
            N = 100;

            Q = Matrix.Build.Diagonal(n, n, q * q); //covariance of process
            NotifyChanged("Q");
            R = Matrix.Build.Dense(1, 1, r * r); //covariance of measurement  
            f = new FEquation(); //nonlinear state equations
            h = new HEquation(); //measurement equation
            x = q * Matrix.Build.Random(1, 1);  //s + q * Matrix.Build.Random(1, 1); //initial state with noise
            P = Matrix.Build.Diagonal(n, n, 1); //initial state covariance


            var xV = Matrix.Build.Dense(n, N, 0); //Estimate
            var zV = Matrix.Build.Dense(1, N, 0); //measurement


            for (int k = 1; k < N; k++)
            {
                Matrix<double> z = ProcessBuilder.SineWave(k, r);
                //measurments
           
                Matrix<double>[] x_and_P = filter.Update(f, x, P, h, z, Q, R);                //ukf 
                x = x_and_P[0];
                P = x_and_P[1];
               
                Measurements.Add(new Measurement() { Value = z[0, 0], Time = TimeSpan.FromSeconds(k) });
                Estimates.Add(new Measurement() { Value = x_and_P[0][0, 0], Time = TimeSpan.FromSeconds(k) ,Variance= x_and_P[1][0, 0] });
            }



  

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


    public static class ProcessBuilder
    {
        public static Matrix<double> SineWave(int iteration, double Noise)
        {
            return Matrix.Build.Dense(1, 1, Math.Sin(iteration * 3.14 * 5 / 180)).Add(Matrix.Build.Random(1, 1).Multiply(Noise));


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




