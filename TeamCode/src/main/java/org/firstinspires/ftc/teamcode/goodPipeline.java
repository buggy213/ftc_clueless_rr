package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class goodPipeline {
    public enum Position {
        Unknown, Left, Middle, Right
    }

    public static enum runCode {
        runCodeCommand();

    }

    /*public static enum MineralProcessing(
    //Mat input)

    //{
        // TODO Auto-generated method stub
        //Mat mat = Mat.eye(3, 3, CvType.CV_8UC1);
        //String position;

        Mat input = Imgcodecs.imread("C:\\Users\\houj0\\Downloads\\MINERAL1.jpg");
        Mat goldInput = input.clone();
        Position pos = Position.Unknown;
        Mat silverInput = input.clone();
        int depth = input.depth();
        long eleSize = input.elemSize();
        Size s = input.size();
        double imageWidth = s.width;
        //System.out.println("depth = " + depth + " eleSize = " + eleSize);
        GoldPipeline gold = new GoldPipeline();
        gold.process(goldInput);
        //SilverPipeline silver = new SilverPipeline();
        //silver.process(silverInput);
        //Mat silvermask = silver.hsvThresholdOutput();
        Mat goldmask = gold.hsvThresholdOutput();
        List<MatOfPoint> contoursSilver = new ArrayList<>();
        List<MatOfPoint> contoursGold = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat hierarchy2 = new Mat();
        //Imgproc.findContours(silvermask, contoursSilver, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursSilver, -1, new Scalar(230, 70, 70), 2);
        //HighGui.imshow("Hi",input);
        Imgproc.findContours(goldmask, contoursGold, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursGold, -1, new Scalar(230, 70, 70), 2);
        Rect chosenGoldRect = null;

        double chosenGoldScore = Integer.MAX_VALUE;


        MatOfPoint2f approxCurve = new MatOfPoint2f();

        List<Rect> goldRectangles = new ArrayList<>();
        double largestArea = 0;
        double xCenterLargest = 0;
        for (MatOfPoint c : contoursGold) {

            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());


            //Processing on mMOP2f1 which is in type MatOfPoint2f

            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;

            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);


            //Convert back to MatOfPoint

            MatOfPoint points = new MatOfPoint(approxCurve.toArray());


            // Get bounding rect of contour

            Rect rect = Imgproc.boundingRect(points);
            Size ss = rect.size();
            Point bottomRight = rect.br();
            Point topLeft = rect.tl();
            //System.out.print(bottomRight.toString());
            //System.out.print(rect.toString());
            //System.out.print(topLeft.toString());
            if (rect.area() > largestArea) {
                xCenterLargest = topLeft.x + ss.width / 2;
                largestArea = rect.area();
            }
        }
        if (xCenterLargest < (imageWidth * 1 / 3)) {
            pos = Position.Left;
            if ((imageWidth * 2 / 3) < xCenterLargest) {
                pos = Position.Middle;
            } else {
                pos = Position.Right;
            }
        }


        //Imgproc.findContours(silvermask, contoursSilver, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(input,contoursSilver,-1,new Scalar(230,70,70),2);
        //HighGui.imshow("Hi",input);


        //MatOfPoint2f approxCurve = new MatOfPoint2f();


        //for(MatOfPoint c : contoursSilver) {

        //MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());


        //Processing on mMOP2f1 which is in type MatOfPoint2f

        //double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;

        //Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);


        //Convert back to MatOfPoint

        //MatOfPoint points = new MatOfPoint(approxCurve.toArray());


        // Get bounding rect of contour

        //Rect rect = Imgproc.boundingRect(points);
        //Size ss = rect.size();
        //Point bottomRight = rect.br();
        //Point topLeft = rect.tl();
        //System.out.print(bottomRight.toString());
        //System.out.print(rect.toString());
        //System.out.print(topLeft.toString());
        //}


    }*/

    public Position runCode(Mat input) {
        // TODO Auto-generated method stub
        //Mat mat = Mat.eye(3, 3, CvType.CV_8UC1);
        //String position;

        Position pos = Position.Unknown;
        Mat goldInput = input.clone();
        Mat silverInput = input.clone();
        int depth = input.depth();
        long eleSize = input.elemSize();
        Size s = input.size();
        double imageWidth = s.width;
        //System.out.println("depth = " + depth + " eleSize = " + eleSize);
        GoldPipeline gold = new GoldPipeline();
        gold.process(goldInput);
        //SilverPipeline silver = new SilverPipeline();
        //silver.process(silverInput);
        //Mat silvermask = silver.hsvThresholdOutput();
        Mat goldmask = gold.hsvThresholdOutput();
        List<MatOfPoint> contoursSilver = new ArrayList<>();
        List<MatOfPoint> contoursGold = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat hierarchy2 = new Mat();
        //Imgproc.findContours(silvermask, contoursSilver, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursSilver, -1, new Scalar(230, 70, 70), 2);
        //HighGui.imshow("Hi",input);
        Imgproc.findContours(goldmask, contoursGold, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursGold, -1, new Scalar(230, 70, 70), 2);
        Rect chosenGoldRect = null;

        double chosenGoldScore = Integer.MAX_VALUE;


        MatOfPoint2f approxCurve = new MatOfPoint2f();

        List<Rect> goldRectangles = new ArrayList<>();
        double largestArea = 0;
        double xCenterLargest = 0;
        for (MatOfPoint c : contoursGold) {

            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());


            //Processing on mMOP2f1 which is in type MatOfPoint2f

            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;

            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);


            //Convert back to MatOfPoint

            MatOfPoint points = new MatOfPoint(approxCurve.toArray());


            // Get bounding rect of contour

            Rect rect = Imgproc.boundingRect(points);
            Size ss = rect.size();
            Point bottomRight = rect.br();
            Point topLeft = rect.tl();
            //System.out.print(bottomRight.toString());
            //System.out.print(rect.toString());
            //System.out.print(topLeft.toString());
            if (rect.area() > largestArea) {
                xCenterLargest = topLeft.x + ss.width / 2;
                largestArea = rect.area();
            }
        }
        if (xCenterLargest < (imageWidth * 1 / 3)) {
            pos = Position.Left;
            if ((imageWidth * 2 / 3) < xCenterLargest) {
                pos = Position.Middle;
            } else {
                pos = Position.Right;
            }
        }


        //Imgproc.findContours(silvermask, contoursSilver, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(input,contoursSilver,-1,new Scalar(230,70,70),2);
        //HighGui.imshow("Hi",input);


        //MatOfPoint2f approxCurve = new MatOfPoint2f();


        //for(MatOfPoint c : contoursSilver) {

        //MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());


        //Processing on mMOP2f1 which is in type MatOfPoint2f

        //double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;

        //Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);


        //Convert back to MatOfPoint

        //MatOfPoint points = new MatOfPoint(approxCurve.toArray());


        // Get bounding rect of contour

        //Rect rect = Imgproc.boundingRect(points);
        //Size ss = rect.size();
        //Point bottomRight = rect.br();
        //Point topLeft = rect.tl();
        //System.out.print(bottomRight.toString());
        //System.out.print(rect.toString());
        //System.out.print(topLeft.toString());
        //}


        return pos;
    }

    public void runCodeCommand() {
        Mat input10 = Imgcodecs.imread("C:\\Users\\houj0\\Downloads\\MINERAL1.jpg");
        runCode(input10);
        Log.d("debug message",runCode(input10)+" ");
        if(runCode(input10) == Position.Left){
            //Condition example
        }
    }

}






