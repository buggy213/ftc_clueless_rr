package org.firstinspires.ftc.teamcode.autonomous.vision;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CalibratingPipeline extends OpenCVPipeline {
    public Mat rgbaCapture;
    private boolean captureFlag;

    private Threshold threshold;

    private Mat hsv = new Mat();
    private Mat mask = new Mat();

    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        if (captureFlag) {
            rgbaCapture = rgba.clone();
            captureFlag = false;
        }
        if (threshold != null) {
            Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
            threshold.threshold(hsv, mask);

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            MatOfPoint[] contourArray = contours.toArray(new MatOfPoint[0]);
            Arrays.sort(contourArray, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                    return (int)(Imgproc.contourArea(matOfPoint) - Imgproc.contourArea(t1));
                }
            });

            Imgproc.drawContours(rgba, Collections.singletonList(contourArray[0]),0, new Scalar(0, 255, 0));
        }
        return rgba;
    }

    public void capture() {
        captureFlag = true;
    }

    public void setThreshold(Threshold threshold) {
        this.threshold = threshold;
    }
}
