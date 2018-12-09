package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Localizer;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autonomous.actions.SamplingArmAction;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;
import org.firstinspires.ftc.teamcode.motionplanningtest.drive.ComplementaryVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.motionplanningtest.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.motionplanningtest.util.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.motionplanningtest.util.DashboardUtil;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.autonomous.SelectParameters.matchParameters;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Main Autonomous")
@Config
public class Autonomous extends LinearOpMode {

    VuforiaLocalizer vuforia;

    public static boolean debug = true;
    public static int mineral = 0;

    Map<String, Map<String, Trajectory>> trajectories;


    void loadTrajectories() {
        trajectories = new HashMap<>();
        try {
            for (SelectParameters.StartingPosition position : SelectParameters.StartingPosition.values()) {
                Map<String, Trajectory> subMap = new HashMap<>();
                for (SelectParameters.SamplingPosition samplingPosition : SelectParameters.SamplingPosition.values()) {
                    Trajectory t = AssetsTrajectoryLoader.load((position + "_" + samplingPosition).toLowerCase());
                    subMap.put(samplingPosition.name(), t);
                }
                trajectories.put(position.name(), subMap);
            }
        }
        catch (IOException e) {
            RobotLog.e(e.toString());
            requestOpModeStop();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        RoverRuckusMecanumDriveREVOptimized drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap);
        RobotHardware rw = new RobotHardware(hardwareMap);
        SelectParameters.Parameters matchParameters = SelectParameters.matchParameters;

        /*VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AcJSEaj/////AAABmZmYKDlSrk/tqCYXQbWF2zIZKePATp9JlOVeqyDFYAOLOJcwm5eO/EyXGAUCjoeJ2LwBWY5JdtBWhaRt3GVVLchqkKSMg8nrFAyAoQUAhLx1i+/9pcqq0Z7HMByQMzWK216ENi7sQ2GrKnETBbbt+82fyitozuTSFr1LAbIsbDd4IjsTmK7CRBhG0Ns9cOr7+prlgNiGHKYhvRwfvV8asHK76mCTPInkYhHMoC8bm+00Z3vzlFhNrDjOu4LpjVXH8KD2ksauFeM1lt3OOLqebuz4IbYkxTN33ZPU1pqJb9qYFaWIMO/7q6XTXMr9fO0Jm0zFt6N+yCe4lWq1UHrnG/MiK8CS30m/eLxItELpmHj7";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);*/

        SamplingPipeline pipeline = new SamplingPipeline();
        pipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());


        // ComplementaryVuforiaLocalizer localizer = new ComplementaryVuforiaLocalizer(vuforia, VuforiaLocalizer.CameraDirection.BACK, drive.getLocalizer());

        // TODO make localizer find initial position (instead of relying on hardcoded values)
        drive.getLocalizer().setPoseEstimate(matchParameters.startingPosition.startingPosition);
        while (!isStarted()) {
            drive.updatePoseEstimate();
            telemetry.addData("Ready", "ready");
            telemetry.addData("Heading", radToDeg(drive.getExternalHeading()));
            telemetry.addData("Starting Position", matchParameters.startingPosition.startingPosition.toString());
            telemetry.update();
        }

        // rw.resetEncoders();

        Trajectory trajectory;
        TrajectoryBuilderExtended builder;
        while (opModeIsActive()) {
            if (debug) {
                matchParameters.mineralConfiguration = SelectParameters.SamplingPosition.values()[mineral];
            }
            builder = freshTrajectoryBuilder(drive);
            builder = builder.strafeRight(Math.sqrt(200));
            trajectory = builder.build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive, trajectory);


            builder = freshTrajectoryBuilder(drive);

            switch(matchParameters.startingPosition) {
                case RED_FACING_DEPOT:
                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            builder = builder.splineTo(new Pose2d(new Vector2d(46, -26), degToRad(-45))).splineTo(new Pose2d(new Vector2d(54.5, -46), degToRad(-90)));
                            builder = builder.splineTo(new Pose2d(new Vector2d(30, -63), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-20, -63), new ConstantInterpolator(degToRad(-180)));
                            break;
                        case CENTER:
                            builder = builder.turnTo(degToRad(-45)).splineTo(new Pose2d(new Vector2d(50, -57), degToRad(-90)))
                                    .splineTo(new Pose2d(new Vector2d(40, -62.5), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-16, -62.5));
                            break;
                        case RIGHT:
                            builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -57), 0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-15, -57));
                    }
                    break;
                case RED_FACING_CRATER:
                    SamplingArmAction armAction;
                    Thread armThread;
                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            builder = builder.splineTo(new Pose2d(new Vector2d(-17, -28), degToRad(-45))).splineTo(new Pose2d(new Vector2d(15, -61), degToRad(0))).turnTo(0);
                            builder = builder.lineTo(new Vector2d(49, -62.5)).turnTo(degToRad(-180)).lineTo(new Vector2d(-20, -62.5));

                            //waitAndDisplayTrajectory(5, dashboard, builder);
                            armAction = new SamplingArmAction(1.5,5,rw);
                            armThread = new Thread(armAction);
                            armThread.start();
                            break;
                        case CENTER:
                            builder = builder.splineTo(new Pose2d(new Vector2d(-35, -26), degToRad(-45))).turnTo(degToRad(-45)).lineTo(new Vector2d(-25, -37), new ConstantInterpolator(degToRad(-45)))
                                    .splineTo(new Pose2d(new Vector2d(10, -62.5), 0)).turnTo(0).lineTo(new Vector2d(48, -62.5), new ConstantInterpolator(0)).turnTo(degToRad(-180)).lineTo(new Vector2d(-20,-60), new ConstantInterpolator(degToRad(-180)));


                            // SamplingArmAction armActionMiddle = new SamplingArmAction(0,0,rw);
                            // Thread armMiddle = new Thread(armActionMiddle);
                            // armMiddle.start();
                            break;
                        case RIGHT:
                            builder = builder.splineTo(new Pose2d(new Vector2d(-39, -14), degToRad(-45))).splineTo(new Pose2d(new Vector2d(26, -60), 0)).lineTo(new Vector2d(48, -60), new ConstantInterpolator(0));
                            builder = builder.lineTo(new Vector2d(-20, -60), new ConstantInterpolator(0));
                    }
                    break;
                case BLUE_FACING_DEPOT:
                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            builder = builder.splineTo(new Pose2d(new Vector2d(-46, 26), degToRad(135))).splineTo(new Pose2d(new Vector2d(-54.5, 46), degToRad(90)));
                            builder = builder.splineTo(new Pose2d(new Vector2d(-30, 63), degToRad(0))).turnTo(degToRad(0)).lineTo(new Vector2d(20, 63), new ConstantInterpolator(degToRad(0)));
                            break;
                        case CENTER:
                            builder = builder.turnTo(degToRad(135)).splineTo(new Pose2d(new Vector2d(-50, 50), degToRad(90)))
                                    .splineTo(new Pose2d(new Vector2d(-40, 60), degToRad(0))).turnTo(degToRad(0)).lineTo(new Vector2d(16, 60));
                            break;
                        case RIGHT:
                            builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -57), 0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-15, -57));
                    }
                    break;
            }

            trajectory = builder.build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive, trajectory);

            /*switch (matchParameters.mineralConfiguration) {
                case LEFT:
                    builder = builder.movePolarRelative(32, -45);
                    break;
                case CENTER:
                    builder = builder.strafeRight(17.5);
                    builder = builder.forward(4);
                    break;
                case RIGHT:
                    builder = builder.movePolarRelative(32, -135);
                    break;
            }
            trajectory = builder.build();

            drive.followTrajectory(trajectory);
            rw.samplingServo.setPosition(SAMPLING_SERVO_UP);

            waitForTrajectoryFinish(drive);

            rw.samplingServo.setPosition(SAMPLING_SERVO_DOWN);

            builder = freshTrajectoryBuilder(drive);
            switch (matchParameters.mineralConfiguration) {
                case LEFT:
                    builder = builder.back(5);
                    break;
                case CENTER:
                    builder = builder.back(7);
                    break;
                case RIGHT:
                    builder = builder.forward(5);
                    break;
            }
            trajectory = builder.build();

            drive.followTrajectory(trajectory);

            waitForTrajectoryFinish(drive);

            rw.samplingServo.setPosition(SAMPLING_SERVO_UP);


            builder = freshTrajectoryBuilder(drive);

            switch (matchParameters.startingPosition) {
                case RED_FACING_DEPOT:
                    builder = builder.reverse().lineTo(new Vector2d(0, -60)).reverse().turnTo(0).forward(30);
                    trajectory = builder.build();

                    break;
                case RED_FACING_CRATER:
                    builder.lineTo(new Vector2d(0, 60), new ConstantInterpolator(3/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(60, 48), 1/2 * Math.PI));
                    break;
                case BLUE_FACING_DEPOT:
                    builder.lineTo(new Vector2d(0, -60), new ConstantInterpolator(7/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(-60, -48), 3/2 * Math.PI));
                    break;
                case BLUE_FACING_CRATER:
                    builder.lineTo(new Vector2d(0, -60), new ConstantInterpolator(5/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(-60, -48), 3/2 * Math.PI));
                    break;
            }

            trajectory = builder.build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive);

            trajectory = freshTrajectoryBuilder(drive).back(50).build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive);
            */
            requestOpModeStop();
        }
    }

    TrajectoryBuilderExtended freshTrajectoryBuilder(RoverRuckusMecanumDriveREVOptimized drive) {
        return new TrajectoryBuilderExtended(drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
    }

    void waitForTrajectoryFinish(RoverRuckusMecanumDriveREVOptimized drive, Trajectory trajectory) {
        while (opModeIsActive() && drive.isFollowingTrajectory()) {
            if (debug) {
                drive.displayPosition(trajectory);
            }
            drive.update();
        }
    }

    void simpleGoldDetect(Mat img) {

    }

    double degToRad(double deg) {
        return Math.PI * deg / 180;
    }

    double radToDeg(double rad) {
        return rad * 180 / Math.PI;
    }

    void waitAndDisplayTrajectory(double time, FtcDashboard dashboard, TrajectoryBuilderExtended trajectoryBuilder){
        Trajectory trajectory = trajectoryBuilder.build();
        resetStartTime();
        while(getRuntime() < time) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

            dashboard.sendTelemetryPacket(packet);
        }
    }


}
