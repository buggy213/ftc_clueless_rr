package org.firstinspires.ftc.teamcode.autonomous.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

public class Threshold {
    private Scalar[] lowerBounds;
    private Scalar[] upperBounds;
    private Mat temp = new Mat();

    public Scalar[] getLowerBounds() {
        return lowerBounds;
    }

    public Scalar[] getUpperBounds() {
        return upperBounds;
    }

    public Threshold() {
        // Change these values here based on lighting of venue
        lowerBounds = new Scalar[] {
                new Scalar(18, 128, 128)
        };
        upperBounds = new Scalar[] {
                new Scalar(54, 255, 255)
        };
    }

    public Threshold(Scalar lower, Scalar upper) {
        this(new Scalar[] {lower}, new Scalar[] {upper});
    }

    public Threshold(Scalar[] lowerBounds, Scalar[] upperBounds) {
        this.lowerBounds = lowerBounds;
        this.upperBounds = upperBounds;
    }

    public Mat threshold(Mat input, Mat mask) {
        Core.inRange(input, lowerBounds[0], upperBounds[0], mask);
        for (int i = 1; i < lowerBounds.length; i++) {
            Core.inRange(input, lowerBounds[i], upperBounds[i], temp);
            Core.bitwise_or(temp, mask, mask);
        }
        return mask;
    }
}
