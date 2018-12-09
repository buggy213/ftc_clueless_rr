package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Util;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
@Config
public class SamplingPipeline extends OpenCVPipeline {

    Mat hsv = new Mat();
    Mat gold = new Mat();
    public static double idealArea = 600;
    double maxAreaDeviation = 150;
    double areaWeight = 1;

    double maxSamples = 5;

    List<MatOfPoint> matOfPoints;
    Mat hierarchy = new Mat();
    MatOfPoint bestContour;

    public static double idealSolidity = 0.8;
    double maxSolidityDeviation = 0.3;
    double solidityWeight = 1;
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        if (rgba == null || rgba.size() == new Size(0,0))
            return rgba;


        matOfPoints = new ArrayList<>();
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(18, 128, 128), new Scalar(54, 255, 255), gold);
        Imgproc.findContours(gold, matOfPoints, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxScore = -100;
        bestContour = null;


        for (MatOfPoint contour : matOfPoints) {
            double area = Imgproc.contourArea(contour);
            double areaScore = getAreaScore(area);
            double solidityScore = getSolidityScore(contour);
            double score = areaScore + solidityScore;
            if (area > maxScore) {
                maxScore = area;
                bestContour = contour;
            }
        }
        if (bestContour != null) {
            Rect boundingBox = Imgproc.boundingRect(bestContour);
            Imgproc.rectangle(rgba, boundingBox.tl(), boundingBox.br(), new Scalar(0, 0, 255), 5);
            Imgproc.putText(rgba, String.valueOf(maxScore), boundingBox.tl(), 0, 1, new Scalar(0, 255, 0));
            if (samples.size() > maxSamples && samples.size() > 0)
                samples.removeFirst();
            samples.add(determinePosition(bestContour, rgba.size().width, rgba));
            plurality = findPlurality(samples);
        }

        hierarchy.release();
        for (MatOfPoint contour : matOfPoints) {
            contour.release();
        }
        hsv.release();
        gold.release();
        gray.release();
        return rgba;
    }

    public SelectParameters.SamplingPosition determinePosition(MatOfPoint contour, double imageX, Mat rgba) {
        if (Imgproc.contourArea(contour) < 5000) {
            return SelectParameters.SamplingPosition.RIGHT;
        }

        Moments contourMoments = Imgproc.moments(contour);
        double centroidX = contourMoments.m10 / contourMoments.m00;
        double centroidY = contourMoments.m01 / contourMoments.m00;
        Imgproc.circle(rgba, new Point(centroidX, centroidY), 3, new Scalar(255,0,0), 3);

        if (centroidX < imageX / 2) {
            return SelectParameters.SamplingPosition.LEFT;
        }
        else {
            return SelectParameters.SamplingPosition.CENTER;
        }
    }


    public LinkedList<SelectParameters.SamplingPosition> samples = new LinkedList<>();
    public SelectParameters.SamplingPosition plurality = SelectParameters.SamplingPosition.RIGHT;

    public double getAreaScore(double area) {
        double deviance = 0;
        if (area > idealArea + maxAreaDeviation) {
            deviance = area - (idealArea + maxAreaDeviation);
            deviance *= areaWeight;
        }
        else if (area < idealArea - maxAreaDeviation) {
            deviance = (idealArea + maxAreaDeviation) - area;
            deviance *= areaWeight;
        }

        return deviance;
    }

    public double getSolidityScore(MatOfPoint contour) {
        Rect boundingBox = Imgproc.boundingRect(contour);
        double solidity = Imgproc.contourArea(contour) / (boundingBox.height * boundingBox.width);
        double deviance = 0;
        if (solidity > idealSolidity + maxSolidityDeviation) {
            deviance = solidity - (idealSolidity + maxSolidityDeviation);
            deviance *= solidityWeight;
        }
        else if (solidity < idealSolidity + maxSolidityDeviation) {
            deviance = (idealSolidity + maxSolidityDeviation) - solidity;
            deviance *= solidityWeight;
        }

        return deviance;
    }

    public SelectParameters.SamplingPosition findPlurality(List<SelectParameters.SamplingPosition> nums) {
        SelectParameters.SamplingPosition candidate = SelectParameters.SamplingPosition.LEFT;
        int count = 0;
        for (SelectParameters.SamplingPosition num : nums) {
            if (count == 0)
                candidate = num;
            if (num == candidate)
                count++;
            else
                count--;
        }
        return candidate;
    }
}
