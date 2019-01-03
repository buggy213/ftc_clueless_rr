package org.firstinspires.ftc.teamcode.autonomous.vision;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AutoCalibrate {

    private final int STEP_SIZE = 1;
    private final int TOLERANCE = 2;
    private final double MAX_LOSS = 0.25;

    public Threshold calibrate(Mat input, Mat mask, int toHSV, int toGray) {
        Imgproc.cvtColor(input, input, toHSV);
        // Imgproc.cvtColor(mask, mask, toGray);

        if (mask.size() != input.size()) {
            // Mask is too big
            Imgproc.resize(mask, mask, input.size());
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            RobotLog.w("Too many contours in mask");
        }
        MatOfPoint contour = contours.get(0);
        double[] hueBounds = optimizeChannel(0, contour, input, mask);
        double[] satBounds = optimizeChannel(1, contour, input, mask);
        double[] valBounds = optimizeChannel(2, contour, input, mask);

        RobotLog.i("Thresholding complete");
        RobotLog.i(Arrays.toString(hueBounds));
        RobotLog.i(Arrays.toString(satBounds));
        RobotLog.i(Arrays.toString(valBounds));

        hierarchy.release();

        Scalar lowerBound = new Scalar(hueBounds[0], satBounds[0], valBounds[0]);
        Scalar upperBound = new Scalar(hueBounds[1], satBounds[1], valBounds[1]);
        return new Threshold(lowerBound, upperBound);
    }

    private double[] optimizeChannel(int channel, MatOfPoint contour, Mat image, Mat mask) {
        int lower = 0;
        int upper = 255;

        double area = Imgproc.contourArea(contour);

        while (testAmountLost(lower, 255, channel, 3, area, image, mask)){
            if (lower > 255) {
                lower = 0;
                break;
            }
            lower += STEP_SIZE;
        }
        while (testAmountLost(0, upper, channel, 3, area, image, mask)){
            if (upper > 255) {
                upper = 0;
                break;
            }
            upper -= STEP_SIZE;
        }

        return new double[] {Range.clip(lower - TOLERANCE, 0, 255), Range.clip(upper + TOLERANCE, 0, 255)};
    }

    private boolean testAmountLost(int lower, int upper, int channel, int channels, double contourSize, Mat image, Mat mask) {
        double[] lowerScalarValues = new double[channels];
        double[] upperScalarValues = new double[channels];
        Arrays.fill(lowerScalarValues, 0);
        Arrays.fill(upperScalarValues, 255);

        lowerScalarValues[channel] = lower;
        upperScalarValues[channel] = upper;

        Scalar lowerScalar = new Scalar(lowerScalarValues);
        Scalar upperScalar = new Scalar(upperScalarValues);

        Mat testMask = new Mat();

        Core.inRange(image, lowerScalar, upperScalar, testMask);

        Core.bitwise_and(testMask, mask, testMask);

        int contoursSum = Core.countNonZero(testMask);
        testMask.release();

        return (contoursSum / contourSize) > (1 - MAX_LOSS / 2);
    }
}
