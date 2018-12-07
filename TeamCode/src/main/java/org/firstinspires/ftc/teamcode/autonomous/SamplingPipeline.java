package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.Util;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SamplingPipeline extends OpenCVPipeline {

    Mat hsv = new Mat();
    Mat gold = new Mat();
    double idealArea;
    double maxAreaDeviation;
    double areaWeight;

    double idealSolidity;
    double maxSolidityDeviation;
    double solidityWeight;
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        List<MatOfPoint> matOfPoints = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(27, 128, 128), new Scalar(45, 255, 255), gold);
        Imgproc.findContours(gold, matOfPoints, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxScore = -100;
        MatOfPoint bestContour = null;


        for (MatOfPoint contour : matOfPoints) {
            double area = Imgproc.contourArea(contour);
            double areaScore = getAreaScore(area);
            double solidityScore = getSolidityScore(contour);
            double score = areaScore + solidityScore;
            if (score > maxScore) {
                maxScore = score;
                bestContour = contour;
            }
        }

        Imgproc.drawContours(rgba, Arrays.asList(bestContour), 0, new Scalar(255, 0, 0));
        Imgproc.putText(rgba, String.valueOf(maxScore), Imgproc.boundingRect(bestContour).tl(), 0, 1, new Scalar(0, 255, 0));

        return rgba;
    }

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
}
