# UKFSharp [![Build Status](https://travis-ci.org/prozoroff/UKFSharp.svg?branch=master)](https://travis-ci.org/prozoroff/UKFSharp)
Simple C# implementation of Unscented Kalman Filter using Math.Net Numerics library.

The Unscented Kalman Filter (UKF) is a solution to bypass restrictions of highly nonlinear systems. (The well-known Kalman Filter is basically suitable for linear systems.) The idea is to produce several sampling points (Sigma points) around the current state estimate based on its covariance. Then, propagating these points through the nonlinear map to get more accurate estimation of the mean and covariance of the mapping results. In this way, it avoids the need to calculate the Jacobian, hence incurs only the similar computation load as the EKF.

This implementation is a simplified version of UKF formulation, where we assume both the process and measurement noises are additive to avoid augment of state and also to simplify the assumption on nonlinear maps.

Example of usage for sin(x) model:

```
var filter = new UKF();

List<double> measurements = new List<double>();
List<double> estimations = new List<double>();
     
Random rnd = new Random();

for (int k = 0; k < 100; k++)
{
    var noisySin = Math.Sin(k * 3.14 * 5 / 180) + (double)rnd.Next(50) / 100;
    var measurement = new[] { noisySin };
    filter.Update(measurement);

    measurements.Add(measurement);
    estimations.Add(filter.getState()[0]);
}
```

![alt tag](https://raw.githubusercontent.com/prozoroff/UKFSharp/master/Data/result.png)
