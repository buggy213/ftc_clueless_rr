package org.firstinspires.ftc.teamcode.autonomous.vision;

import com.acmerobotics.dashboard.config.Config;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.autonomous.parameters.Mineral;
import org.firstinspires.ftc.teamcode.shared.RobotConstants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
@Config
public class SamplingPipeline extends OpenCVPipeline {

    Mat hsv = new Mat();
    Mat gold = new Mat();
    public static double idealArea = 600;
    double maxAreaDeviation = 150;
    double areaWeight = 1;

    double maxSamples = 20;

    List<MatOfPoint> matOfPoints;
    Mat hierarchy = new Mat();
    MatOfPoint bestContour;

    public static double idealSolidity = 0.8;
    double maxSolidityDeviation = 0.3;
    double solidityWeight = 1;
    private Mat cropped;
    final int tlx = 30;
    final int tly = 140;
    final int brx = 1100;
    final int bry = 464;

    final Point tl = new Point(tlx, tly);
    final Point br = new Point(brx, bry);


    public static int KERNEL_SIZE = 3;
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2*KERNEL_SIZE + 1, 2*KERNEL_SIZE+1));


    Threshold goldThreshold;
    @Override
    public void enable() {
        super.enable();
        if (AutoCalibrateOpMode.load() == null) {
            goldThreshold = new Threshold();
        }
        else {
            goldThreshold = AutoCalibrateOpMode.load();
        }
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        if (rgba == null || rgba.size() == new Size(0,0))
            return rgba;

        if (rgba.size().width == 1280 || rgba.size().height == 720) {
            cropped = rgba.submat(new Rect(tl, br));
        }
        else {
            cropped = rgba;
        }

        matOfPoints = new ArrayList<>();
        Imgproc.blur(cropped, cropped, new Size(4, 4));
        Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_RGB2HSV);

        goldThreshold.threshold(hsv, gold);

        Imgproc.dilate(gold, gold, kernel);
        Imgproc.erode(gold, gold, kernel);

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

        Mineral mineralPosition = Mineral.RIGHT;

        if (bestContour != null) {
            Rect boundingBox = Imgproc.boundingRect(bestContour);
            Imgproc.rectangle(rgba, addPoints(boundingBox.tl(), tl), addPoints(boundingBox.br(), tl), new Scalar(0, 0, 255), 5);
            Imgproc.putText(rgba, String.valueOf(maxScore), boundingBox.tl(), 0, 1, new Scalar(0, 255, 0));

            mineralPosition = determinePosition(bestContour, cropped.size().width, rgba);
        }
        if (samples.size() > maxSamples && samples.size() > 0)
            samples.removeFirst();
        samples.add(mineralPosition);
        plurality = findPlurality(samples);

        Imgproc.rectangle(rgba, tl, br, new Scalar(255, 0, 0), 5);

        hierarchy.release();
        for (MatOfPoint contour : matOfPoints) {
            contour.release();
        }
        hsv.release();
        gold.release();
        gray.release();
        return rgba;
    }

    private Point addPoints(Point p1, Point p2) {
        return new Point(p1.x + p2.x, p1.y + p2.y);
    }

    public Mineral determinePosition(MatOfPoint contour, double imageX, Mat rgba) {
        if (Imgproc.contourArea(contour) < RobotConstants.CONTOUR_MIN_AREA) {
            return Mineral.RIGHT;
        }

        Moments contourMoments = Imgproc.moments(contour);
        double centroidX = contourMoments.m10 / contourMoments.m00;
        double centroidY = contourMoments.m01 / contourMoments.m00;
        Imgproc.circle(rgba, addPoints(new Point(centroidX, centroidY), tl), 3, new Scalar(255,0,0), 3);

        if (centroidX < imageX / 2) {
            return Mineral.LEFT;
        }
        else {
            return Mineral.CENTER;
        }
    }


    public LinkedList<Mineral> samples = new LinkedList<>();
    public Mineral plurality = Mineral.RIGHT;

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

    public Mineral findPlurality(List<Mineral> nums) {
        Mineral candidate = Mineral.LEFT;
        int count = 0;
        for (Mineral num : nums) {
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
