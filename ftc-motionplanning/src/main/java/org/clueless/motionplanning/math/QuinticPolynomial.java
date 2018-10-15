package org.clueless.motionplanning.math;
import Jama.LUDecomposition;
import Jama.Matrix;

// See https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Quintic_Splines_for_FTC.pdf

public class QuinticPolynomial implements ParametricEquation{
    private static double[][] COEFF_MATRIX = {
            {0,0,0,0,0,1},
            {0,0,0,0,1,0},
            {0,0,0,2,0,0},
            {1,1,1,1,1,1},
            {5,4,3,2,1,0},
            {20,12,6,2,0,0}
    };

    // Coefficients of the polynomial
    double a, b, c, d, e, f;

    public QuinticPolynomial(double start, double startDerivative, double startSecondDerivative,
                             double end, double endDerivative, double endSecondDerivative) {
        Matrix coeff = new Matrix(COEFF_MATRIX);
        Matrix target = new Matrix(new double[][]{{start, startDerivative, startSecondDerivative, end, endDerivative, endSecondDerivative}}).transpose();
        LUDecomposition solver = new LUDecomposition(coeff);
        Matrix polynomialCoefficients = solver.solve(target);
        a = polynomialCoefficients.get(0,0);
        b = polynomialCoefficients.get(1,0);
        c = polynomialCoefficients.get(2,0);
        d = polynomialCoefficients.get(3,0);
        e = polynomialCoefficients.get(4,0);
        f = polynomialCoefficients.get(5,0);

    }

    @Override
    public double get(double t) {
        return a * (t * t * t * t * t) + b * (t * t * t * t) + c * (t * t * t) + d * (t * t) + e * (t) + f;
    }

    public double derivative(double t) {
        return 5 * a * Math.pow(t, 4) + 4 * b * Math.pow(t, 3) + 3 * c * Math.pow(t, 2) + 2 * d * t + e;
    }

    public double secondDerivative(double t) {
        return 20 * a * Math.pow(t, 3) + 12 * b * Math.pow(t, 2) + 6 * c * t + 2 * d;
    }

}
