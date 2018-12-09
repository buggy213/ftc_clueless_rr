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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Coordinate_Tetst;
import org.firstinspires.ftc.teamcode.armkinematics.ArmController;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SamplingArmAction;
import org.firstinspires.ftc.teamcode.drivetrain_test.FourWheelMecanumDrivetrain;
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
import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.INTAKE_JOINT_DOWN;
import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.INTAKE_JOINT_MARKER;
import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.INTAKE_JOINT_UP;
import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.LOCK_DISENGAGED;
import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.LOCK_ENGAGED;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Main Autonomous")
@Config
public class Autonomous extends LinearOpMode {

    VuforiaLocalizer vuforia;

    public static boolean debug = false;
    public static int mineral = 0;

    public static int TIME_GOING_DOWN = 2800;

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
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(rw);
        SelectParameters.Parameters matchParameters = SelectParameters.matchParameters;

        ArmController armController = new ArmController(rw, telemetry, true);

        /*VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AcJSEaj/////AAABmZmYKDlSrk/tqCYXQbWF2zIZKePATp9JlOVeqyDFYAOLOJcwm5eO/EyXGAUCjoeJ2LwBWY5JdtBWhaRt3GVVLchqkKSMg8nrFAyAoQUAhLx1i+/9pcqq0Z7HMByQMzWK216ENi7sQ2GrKnETBbbt+82fyitozuTSFr1LAbIsbDd4IjsTmK7CRBhG0Ns9cOr7+prlgNiGHKYhvRwfvV8asHK76mCTPInkYhHMoC8bm+00Z3vzlFhNrDjOu4LpjVXH8KD2ksauFeM1lt3OOLqebuz4IbYkxTN33ZPU1pqJb9qYFaWIMO/7q6XTXMr9fO0Jm0zFt6N+yCe4lWq1UHrnG/MiK8CS30m/eLxItELpmHj7";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);*/

        SamplingPipeline pipeline = new SamplingPipeline();
        pipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        pipeline.enable();

        // ComplementaryVuforiaLocalizer localizer = new ComplementaryVuforiaLocalizer(vuforia, VuforiaLocalizer.CameraDirection.BACK, drive.getLocalizer());

        // TODO make localizer find initial position (instead of relying on hardcoded values)
        drive.getLocalizer().setPoseEstimate(matchParameters.startingPosition.startingPosition);





        while (!isStarted()) {
            drive.updatePoseEstimate();
            System.gc();
            telemetry.addData("Ready", "ready");
            telemetry.addData("Heading", radToDeg(drive.getExternalHeading()));
            telemetry.addData("Starting Position", matchParameters.startingPosition.startingPosition.toString());
            telemetry.addData("Sampling Pipeline Plurality", pipeline.plurality);
            telemetry.update();
        }
        pipeline.disable();
        rw.pawServo.setPosition(LOCK_DISENGAGED);
        SelectParameters.SamplingPosition position = pipeline.plurality;
        Thread.sleep(250);
        // rw.resetEncoders();

        Trajectory trajectory;
        TrajectoryBuilderExtended builder;
        while (opModeIsActive()) {

            if (debug) {
                matchParameters.mineralConfiguration = SelectParameters.SamplingPosition.values()[mineral];
            }
            else {
                matchParameters.mineralConfiguration = position;
            }

            rw.linearSlider.setPower(-1);
            Thread.sleep(800);
            rw.linearSlider.setPower(0.45);
            Thread.sleep(TIME_GOING_DOWN);
            rw.linearSlider.setPower(0);
            Thread.sleep(250);
            rw.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rw.linearSlider.setTargetPosition(rw.linearSlider.getCurrentPosition() + 2400);
            rw.linearSlider.setPower(1);
            Thread.sleep(800);
            AutoMove(-0.25, 0, 250, drivetrain, rw);
            rw.linearSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rw.linearSlider.setPower(-1);
            Thread.sleep(1500);
            rw.linearSlider.setPower(0);
            AutoMove(0.25, 0, 250, drivetrain, rw);

            builder = freshTrajectoryBuilder(drive);
            builder = builder.strafeRight(Math.sqrt(200));
            trajectory = builder.build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive, trajectory);


            builder = freshTrajectoryBuilder(drive);

            rw.intakeJoint.setPosition(INTAKE_JOINT_UP);
            IntakeAction markerAction = null;
            switch(matchParameters.startingPosition) {
                case RED_FACING_DEPOT:

                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            builder = builder.splineTo(new Pose2d(new Vector2d(46, -26), degToRad(-45))).splineTo(new Pose2d(new Vector2d(54.5, -46), degToRad(-90)));
                            builder = builder.splineTo(new Pose2d(new Vector2d(30, -63), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-20, -63), new ConstantInterpolator(degToRad(-180)));
                            markerAction = new IntakeAction(4.3, 5.5, rw);
                            break;
                        case CENTER:
                            builder = builder.turnTo(degToRad(-45)).splineTo(new Pose2d(new Vector2d(50, -57), degToRad(-90)))
                                    .splineTo(new Pose2d(new Vector2d(40, -62.5), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-16, -62.5));
                            markerAction = new IntakeAction(4.3, 5.5, rw);
                            break;
                        case RIGHT:
                            builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -57), 0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-13, -57));
                            markerAction = new IntakeAction(6.5, 7.7, rw);
                            break;
                    }
                    if (markerAction != null) {
                        Thread marker = new Thread(markerAction);
                        marker.start();
                    }

                    break;
                case RED_FACING_CRATER:
                    // SamplingArmAction armAction;
                    // Thread armThread;
                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            //builder = builder.splineTo(new Pose2d(new Vector2d(-17, -28), degToRad(-45))).splineTo(new Pose2d(new Vector2d(15, -61), degToRad(0))).turnTo(0);
                            //builder = builder.lineTo(new Vector2d(49, -62.5)).turnTo(degToRad(-180)).lineTo(new Vector2d(-20, -62.5));

                            //waitAndDisplayTrajectory(5, dashboard, builder);
                            //armAction = new SamplingArmAction(1.5,5,rw);
                            //armThread = new Thread(armAction);
                            //armThread.start();
                            builder = builder.turnTo(degToRad(-90)).lineTo(new Vector2d(-22, -44), new ConstantInterpolator(degToRad(-90))).turnTo(degToRad(-135));
                            break;
                        case CENTER:
                            //builder = builder.splineTo(new Pose2d(new Vector2d(-32, -24), degToRad(-45))).turnTo(degToRad(-45)).lineTo(new Vector2d(-25, -37), new ConstantInterpolator(degToRad(-45)))
                            //        .splineTo(new Pose2d(new Vector2d(10, -62.5), 0)).turnTo(0).lineTo(new Vector2d(48, -62.5), new ConstantInterpolator(0)).turnTo(degToRad(-180)).lineTo(new Vector2d(-20,-60), new ConstantInterpolator(degToRad(-180)));

                            builder = builder.turnTo(degToRad(-135)).lineTo(new Vector2d(-33.5, -33.5), new ConstantInterpolator(degToRad(-135)));

                            // SamplingArmAction armActionMiddle = new SamplingArmAction(0,0,rw);
                            // Thread armMiddle = new Thread(armActionMiddle);
                            // armMiddle.start();
                            break;
                        case RIGHT:
                            // builder = builder.splineTo(new Pose2d(new Vector2d(-39, -14), degToRad(-45))).splineTo(new Pose2d(new Vector2d(26, -60), 0)).lineTo(new Vector2d(48, -62.5), new ConstantInterpolator(0));
                            // builder = builder.lineTo(new Vector2d(-20, -62.5), new ConstantInterpolator(0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-44, -22), new ConstantInterpolator(degToRad(-180))).turnTo(degToRad(-135));
                            break;
                    }
                    break;
                case BLUE_FACING_DEPOT:
                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            builder = builder.splineTo(new Pose2d(new Vector2d(-46, 26), degToRad(135))).splineTo(new Pose2d(new Vector2d(-54.5, 46), degToRad(90)));
                            builder = builder.splineTo(new Pose2d(new Vector2d(-30, 63), degToRad(0))).turnTo(degToRad(0)).lineTo(new Vector2d(20, 63), new ConstantInterpolator(degToRad(0)));
                            markerAction = new IntakeAction(4.3, 5.5, rw);
                            break;
                        case CENTER:
                            builder = builder.turnTo(degToRad(135)).splineTo(new Pose2d(new Vector2d(-50, 50), degToRad(90)))
                                    .splineTo(new Pose2d(new Vector2d(-40, 60), degToRad(0))).turnTo(degToRad(0)).lineTo(new Vector2d(16, 60));
                            markerAction = new IntakeAction(4.3, 5.5, rw);

                            break;
                        case RIGHT:
                            builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -57), 0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-13, -57));
                            markerAction = new IntakeAction(6.5, 7.7, rw);
                            break;
                    }
                    if (markerAction != null) {
                        Thread marker = new Thread(markerAction);
                        marker.start();
                    }
                    break;
                case BLUE_FACING_CRATER:
                    switch(matchParameters.mineralConfiguration) {
                        case LEFT:
                            //builder = builder.splineTo(new Pose2d(new Vector2d(-17, -28), degToRad(-45))).splineTo(new Pose2d(new Vector2d(15, -61), degToRad(0))).turnTo(0);
                            //builder = builder.lineTo(new Vector2d(49, -62.5)).turnTo(degToRad(-180)).lineTo(new Vector2d(-20, -62.5));

                            //waitAndDisplayTrajectory(5, dashboard, builder);
                            //armAction = new SamplingArmAction(1.5,5,rw);
                            //armThread = new Thread(armAction);
                            //armThread.start();
                            builder = builder.turnTo(degToRad(90)).lineTo(new Vector2d(22, 44), new ConstantInterpolator(degToRad(90))).turnTo(degToRad(45));
                            break;
                        case CENTER:
                            //builder = builder.splineTo(new Pose2d(new Vector2d(-32, -24), degToRad(-45))).turnTo(degToRad(-45)).lineTo(new Vector2d(-25, -37), new ConstantInterpolator(degToRad(-45)))
                            //        .splineTo(new Pose2d(new Vector2d(10, -62.5), 0)).turnTo(0).lineTo(new Vector2d(48, -62.5), new ConstantInterpolator(0)).turnTo(degToRad(-180)).lineTo(new Vector2d(-20,-60), new ConstantInterpolator(degToRad(-180)));

                            builder = builder.turnTo(degToRad(45)).lineTo(new Vector2d(33.5, 33.5), new ConstantInterpolator(degToRad(45)));

                            // SamplingArmAction armActionMiddle = new SamplingArmAction(0,0,rw);
                            // Thread armMiddle = new Thread(armActionMiddle);
                            // armMiddle.start();
                            break;
                        case RIGHT:
                            // builder = builder.splineTo(new Pose2d(new Vector2d(-39, -14), degToRad(-45))).splineTo(new Pose2d(new Vector2d(26, -60), 0)).lineTo(new Vector2d(48, -62.5), new ConstantInterpolator(0));
                            // builder = builder.lineTo(new Vector2d(-20, -62.5), new ConstantInterpolator(0));
                            builder = builder.turnTo(0).lineTo(new Vector2d(44, 22), new ConstantInterpolator(degToRad(0))).turnTo(degToRad(45));
                            break;
                    }
                    break;
            }

            trajectory = builder.build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive, trajectory);


            armController.setPositions(750, -750);
            while (opModeIsActive()) {
                armController.updateArmAuto();
            }
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

    public void AutoMove(double speed, double angle, int counts, FourWheelMecanumDrivetrain drivetrain, RobotHardware hw) {
        int initialForward = hw.frontLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive()) {
            int differenceForward = Math.abs(hw.frontLeft.getCurrentPosition() - initialForward);
            int differenceBackward = Math.abs(hw.backLeft.getCurrentPosition() - initialBackward);
            telemetry.addData("d1", differenceForward);
            telemetry.addData("d2", differenceBackward);
            telemetry.update();
            if ((differenceBackward + differenceForward) / 2 > counts) {
                drivetrain.stop();
                break;
            }
        }
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
