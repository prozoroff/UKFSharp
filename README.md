# UKFSharp
Simple C# implementation of Unscented Kalman Filter using Math.Net Numerics library.

The Unscented Kalman Filter (UKF) is a solution to bypass restrictions of highly nonlinear systems. (The well-known Kalman Filter is basically suitable for linear systems.) The idea is to produce several sampling points (Sigma points) around the current state estimate based on its covariance. Then, propagating these points through the nonlinear map to get more accurate estimation of the mean and covariance of the mapping results. In this way, it avoids the need to calculate the Jacobian, hence incurs only the similar computation load as the EKF.

This implementation is a simplified version of UKF formulation, where we assume both the process and measurement noises are additive to avoid augment of state and also to simplify the assumption on nonlinear maps.

Example of usage for sin(x) model:

```
  var filter = new UKF(1, 1);
  var q = 0.05; //std of process 
  var r = 0.3; //std of measurement
  var Q = Matrix.Build.Diagonal(1, 1, q * q); //covariance of process
  var R = Matrix.Build.Dense(1, 1, r * r); //covariance of measurement  
  var f = new FEquation(); //nonlinear state equations
  var h = new HEquation(); //measurement equation
  var x = q * Matrix.Build.Random(1, 1);  //initial state with noise
  var P = Matrix.Build.Diagonal(n, n, 1); //initial state covraiance
  var N = 100; //total dynamic steps

  var xV = Matrix.Build.Dense(n, N, 0); //estmate
  var zV = Matrix.Build.Dense(1, N, 0); //measurement

  for (int k = 1; k < N; k++)
  {
    var measurement = Math.Sin(k*3.14*5/180); 
    var z = Matrix.Build.Dense(1,1,measurement).Add(Matrix.Build.Random(1, 1).Multiply(r)); //measurments
    zV.SetSubMatrix(0, k, z);                                        //save measurment
    var x_and_P = filter.Update(f, x, P, h, z, Q, R);                //ukf 
    x = x_and_P[0];
    P = x_and_P[1];
    xV.SetColumn(k, x.Column(0).ToArray());                          //save estimate
  }
```

![alt tag](https://raw.githubusercontent.com/prozoroff/UKFSharp/master/Data/result.png)
