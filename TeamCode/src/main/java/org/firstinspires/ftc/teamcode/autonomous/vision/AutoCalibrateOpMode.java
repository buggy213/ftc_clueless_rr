package org.firstinspires.ftc.teamcode.autonomous.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.io.FileUtils;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.corningrobotics.enderbots.endercv.OpenCVLoader;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "AutoCalibrate")
public class AutoCalibrateOpMode extends LinearOpMode {

    static {
        try {
            System.loadLibrary("opencv_java3");
        } catch (UnsatisfiedLinkError e) {
            OpenCVLoader.loadOpenCV();
            // pass
        }
    }

    public static Threshold threshold;

    Mat capture = new Mat();

    @Override
    public void runOpMode() {
        AutoCalibrate autoCalibrate = new AutoCalibrate();
        CalibratingPipeline calibratingPipeline = new CalibratingPipeline();
        calibratingPipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        calibratingPipeline.enable();

        Mat mask = Imgcodecs.imread(AppUtil.FIRST_FOLDER + "/mask.png");

        calibratingPipeline.setMask(mask);

        msStuckDetectLoop = 60000;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                calibratingPipeline.capture();
                while(calibratingPipeline.rgbaCapture == null || calibratingPipeline.rgbaCapture.empty()) {

                }
                capture = calibratingPipeline.rgbaCapture;

                Threshold threshold = autoCalibrate.calibrate(capture, mask, Imgproc.COLOR_RGB2HSV, Imgproc.COLOR_BGR2GRAY);
                calibratingPipeline.setThreshold(threshold);
                AutoCalibrateOpMode.threshold = threshold;

                Imgproc.cvtColor(capture, capture, Imgproc.COLOR_RGBA2BGR);
                Imgcodecs.imwrite(AppUtil.FIRST_FOLDER + "/debug.jpg", capture);
                save(threshold);
            }
        }
    }

    public void save(Threshold t) {
        StringBuilder sb = new StringBuilder();
        sb.append(t.getLowerBounds()[0].val[0]);
        sb.append(System.lineSeparator());
        sb.append(t.getLowerBounds()[0].val[1]);
        sb.append(System.lineSeparator());
        sb.append(t.getLowerBounds()[0].val[2]);
        sb.append(System.lineSeparator());
        sb.append(t.getUpperBounds()[0].val[0]);
        sb.append(System.lineSeparator());
        sb.append(t.getUpperBounds()[0].val[1]);
        sb.append(System.lineSeparator());
        sb.append(t.getUpperBounds()[0].val[2]);
        try {
            FileUtils.write(new File(AppUtil.FIRST_FOLDER + "/threshold.thd"), sb.toString(), Charset.defaultCharset());
        }
        catch (IOException e) {
            RobotLog.e("Unable to save");
            RobotLog.e(e.toString());
        }
    }

    public static Threshold load() {
        List<String> thresholdValues;
        try {
            thresholdValues = FileUtils.readLines(new File(AppUtil.FIRST_FOLDER + "/threshold.thd"), Charset.defaultCharset());
            Scalar lowerBounds = new Scalar(Double.valueOf(thresholdValues.get(0)), Double.valueOf(thresholdValues.get(1)), Double.valueOf(thresholdValues.get(2)));

            Scalar upperBounds = new Scalar(Double.valueOf(thresholdValues.get(3)), Double.valueOf(thresholdValues.get(4)), Double.valueOf(thresholdValues.get(5)));
            return new Threshold(lowerBounds, upperBounds);
        }
        catch (IOException e) {
            RobotLog.e("Unable to load");
            RobotLog.e(e.toString());
        }
        return null;
    }
}
