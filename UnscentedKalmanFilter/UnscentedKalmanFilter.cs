﻿using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System;

namespace UnscentedKalmanFilter
{
	public class UKF
	{
		/// <summary>
		/// States number
		/// </summary>
		private int L;  

		/// <summary>
		/// Measurements number
		/// </summary>
		private int m;  

		/// <summary>
		/// The alpha.
		/// </summary>
		private double alpha;  

		/// <summary>
		/// The ki.
		/// </summary>
		private double ki;

		/// <summary>
		/// The beta.
		/// </summary>
		private double beta;

		/// <summary>
		/// Scale factor
		/// </summary>
		private double lambda;

		/// <summary>
		/// Scale factor
		/// </summary>
		private double c; 

		/// <summary>
		/// Means weights
		/// </summary>
		private Matrix<double> Wm; 

		/// <summary>
		/// Covariance weights
		/// </summary>
        private Matrix<double> Wc;

        /// <summary>
        /// Constructor of Unscented Kalman Filter
        /// </summary>
        /// <param name="L">States number</param>
        /// <param name="m">Measurements number</param>
        public UKF(int L, int m)
		{
            this.L = L;
            this.m = m;
            alpha = 1e-3f;
            ki = 0;  
            beta = 2f;
            lambda = alpha * alpha * (L + ki) - L;
            c = L + lambda;

            //weights for means
            Wm = Matrix.Build.Dense(1, (2 * L + 1), 0.5 / c);
            Wm[0, 0] = lambda / c;

            //weights for covariance
            Wc = Matrix.Build.Dense(1, (2 * L + 1));
            Wm.CopyTo(Wc);
            Wc[0, 0] = Wc[0, 0] + 1 - alpha * alpha + beta;

            c = Math.Sqrt(c);	
		}

        /// <summary>
        /// Update process
        /// </summary>
        /// <param name="f">nonlinear state equations</param>
        /// <param name="x_and_P">state and covariance</param>
        /// <param name="h">measurement equation</param>
        /// <param name="z">current measurement</param>
        /// <param name="Q">process noise covariance </param>
        /// <param name="R">measurement noise covariance </param>
        /// <returns></returns>
        public Matrix<double>[] Update(IFunction f, Matrix<double>[] x_and_P, IFunction h, Matrix<double> z, Matrix<double> Q, Matrix<double> R) 
	    {

            //sigma points around x
            Matrix<double> X = GetSigmaPoints(x_and_P[0], x_and_P[1], c);  


            //unscented transformation of process
            // X1=sigmas(x1,P1,c) - sigma points around x1
            //X2=X1-x1(:,ones(1,size(X1,2))) - deviation of X1
            Matrix<double>[] ut_f_matrices = UnscentedTransform(f, X, Wm, Wc, L, Q);  
            Matrix<double> x1 = ut_f_matrices[0];
            Matrix<double> X1 = ut_f_matrices[1];
            Matrix<double> P1 = ut_f_matrices[2];
            Matrix<double> X2 = ut_f_matrices[3];

            //unscented transformation of measurments
            Matrix<double>[] ut_h_matrices = UnscentedTransform(h, X1, Wm, Wc, m, R);  
            Matrix<double> z1 = ut_h_matrices[0];
            Matrix<double> Z1 = ut_h_matrices[1];
            Matrix<double> P2 = ut_h_matrices[2];
            Matrix<double> Z2 = ut_h_matrices[3];

            //transformed cross-covariance
            Matrix<double> P12 = (X2.Multiply(Matrix.Build.Diagonal(Wc.Row(0).ToArray()))).Multiply(Z2.Transpose());

            Matrix<double> K = P12.Multiply(P2.Inverse());

            //state update
            x_and_P[0] = x1.Add(K.Multiply(z.Subtract(z1)));
            //covariance update 
            x_and_P[1] = P1.Subtract(K.Multiply(P12.Transpose()));  

	        return x_and_P;
	    }

        /// <summary>
        /// Unscented Transformation
        /// </summary>
        /// <param name="f">nonlinear map</param>
        /// <param name="X">sigma points</param>
        /// <param name="Wm">Weights for means</param>
        /// <param name="Wc">Weights for covariance</param>
        /// <param name="n">numer of outputs of f</param>
        /// <param name="R">additive covariance</param>
        /// <returns>[transformed mean, transformed smapling points, transformed covariance, transformed deviations</returns>
        private Matrix<double>[] UnscentedTransform(IFunction f, Matrix<double> X, Matrix<double> Wm, Matrix<double> Wc, int n, Matrix<double> R)
        {
            int L = X.ColumnCount;
            Matrix<double> y = Matrix.Build.Dense(n, 1, 0);
            Matrix<double> Y = Matrix.Build.Dense(n, L, 0);

            Matrix<double> row_in_X;
            for (int k = 0; k < L; k++)
            {
                row_in_X = X.SubMatrix(0, X.RowCount, k, 1);
                Y.SetSubMatrix(0, Y.RowCount, k, 1, f.Process(row_in_X));
                y = y.Add(Y.SubMatrix(0, Y.RowCount, k, 1).Multiply(Wm[0,k]));
            }

            Matrix<double> Y1 = Y.Subtract(y.SubMatrix(0,y.RowCount,0,L));
            Matrix<double> P = Y1.Multiply(Matrix.Build.Diagonal(Wc.Row(0).ToArray()));
            P = P.Multiply(Y1.Transpose());
            P = P.Add(R);

            Matrix<double>[] output = { y, Y, P, Y1 };
            return output;
        }

        /// <summary>
        /// Sigma points around reference point
        /// </summary>
        /// <param name="x">reference point</param>
        /// <param name="P">covariance</param>
        /// <param name="c">coefficient</param>
        /// <returns>Sigma points</returns>
        private Matrix<double> GetSigmaPoints(Matrix<double> x, Matrix<double> P, double c) 
	    {
            Matrix<double> A = P.Cholesky().Factor;

	    	A = A.Multiply(c);
	    	A = A.Transpose();

	    	int n = x.RowCount;

	    	Matrix<double> Y = Matrix.Build.Dense(n, n, 1);
	    	for (int j=0; j<n; j++)  
	    	{
	    		Y.SetSubMatrix(0, n-1, j, 1, x);
	    	}

	    	Matrix<double> X = Matrix.Build.Dense(n,(2*n+1));
	    	X.SetSubMatrix(0, n-1, 0, 1, x);

	    	Matrix<double> Y_plus_A = Y.Add(A);	
	    	X.SetSubMatrix(0, n-1, 1, n, Y_plus_A);
	    	
	    	Matrix<double> Y_minus_A = Y.Subtract(A);
	    	X.SetSubMatrix(0, n-1, n+1, n, Y_minus_A);
	    	
	    	return X;
	    }
	}
}
