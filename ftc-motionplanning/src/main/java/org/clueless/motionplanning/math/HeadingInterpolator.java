package org.clueless.motionplanning.math;

public abstract class HeadingInterpolator {
    ParametricEquation curve;

    public HeadingInterpolator(ParametricEquation curve) {
        this.curve = curve;
    }

    abstract boolean respectsDerivativeContinuity(double t);

    abstract double get(double t);

    abstract double derivative(double t);

    abstract double secondDerivative(double t);
}
