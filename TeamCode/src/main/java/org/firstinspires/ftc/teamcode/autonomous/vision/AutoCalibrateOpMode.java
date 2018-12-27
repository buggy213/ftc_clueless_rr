package org.firstinspires.ftc.teamcode.autonomous.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "AutoCalibrate")
public class AutoCalibrateOpMode extends LinearOpMode {

    public static Threshold threshold;

    Mat capture = new Mat();

    @Override
    public void runOpMode() {
        AutoCalibrate autoCalibrate = new AutoCalibrate();
        CalibratingPipeline calibratingPipeline = new CalibratingPipeline();
        calibratingPipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        calibratingPipeline.enable();

        Mat mask = Imgcodecs.imread(AppUtil.FIRST_FOLDER + "/mask.png");

        msStuckDetectLoop = 45000;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                calibratingPipeline.capture();
                while(calibratingPipeline.rgbaCapture.empty()) {

                }
                capture = calibratingPipeline.rgbaCapture;

                Threshold threshold = autoCalibrate.calibrate(capture, mask, Imgproc.COLOR_RGB2HSV, Imgproc.COLOR_BGR2GRAY);
                calibratingPipeline.setThreshold(threshold);
                AutoCalibrateOpMode.threshold = threshold;
            }
        }
    }
}
