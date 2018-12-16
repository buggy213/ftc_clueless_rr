package org.firstinspires.ftc.teamcode.autonomous.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

public class GoldThreshold {
    public Scalar[] lowerBounds;
    public Scalar[] upperBounds;
    Mat temp = new Mat();
    public GoldThreshold() {
        // Change these values here based on lighting of venue
        lowerBounds = new Scalar[] {
                new Scalar(18, 128, 128)
        };
        upperBounds = new Scalar[] {
                new Scalar(54, 255, 255)
        };
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
