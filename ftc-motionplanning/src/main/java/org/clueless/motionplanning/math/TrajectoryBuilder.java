package org.clueless.motionplanning.math;

import java.util.InputMismatchException;

public class TrajectoryBuilder {



    public void addLineSegment(Vector2 point) {

    }

    public void addQuinticSpline(double[] xvals, double[] yvals) {
        if (xvals.length != yvals.length) {
            // x and y value arrays should be the same length
            throw new InputMismatchException();
        }

        QuinticPolynomial[] xPolynomials = new QuinticPolynomial[xvals.length];
        QuinticPolynomial[] yPolynomials = new QuinticPolynomial[yvals.length];

        for (int i = 0; i < xvals.length - 1; i++) {

        }

    }
}
