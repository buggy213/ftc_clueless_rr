package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LiftArmAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LinearSliderAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SamplingArmAction;
import org.firstinspires.ftc.teamcode.autonomous.parameters.Mineral;
import org.firstinspires.ftc.teamcode.autonomous.parameters.Parameters;
import org.firstinspires.ftc.teamcode.autonomous.parameters.SelectParameters;
import org.firstinspires.ftc.teamcode.autonomous.parameters.StartingPosition;
import org.firstinspires.ftc.teamcode.autonomous.vision.SamplingPipeline;
import org.firstinspires.ftc.teamcode.motionplanning.drive.config.DriveConstants;
import org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize.TrajectoryManager;
import org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize.TrajectoryTransform;
import org.firstinspires.ftc.teamcode.motionplanning.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;
import org.firstinspires.ftc.teamcode.shared.RoverRuckusMecanumDriveREVOptimized;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.BACK_SERVO_UP;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.LOCK_DISENGAGED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.BACK_SERVO_DOWN;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Main Autonomous")
@Config
public class Autonomous extends LinearOpMode {

    public static boolean debugPosition = true;
    public static boolean debugMineral = false;
    public static boolean landed = false;
    public static int mineral = 0;

    public static int TIME_GOING_DOWN = 2700;

    public static double STRAFE_AMOUNT = 200;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware rw = new RobotHardware(hardwareMap);
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(rw);

        drivetrain.resetEncoders();

        RoverRuckusMecanumDriveREVOptimized drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap);

        Parameters matchParameters = SelectParameters.getMatchParameters();

        ArmController armController = new ArmController(rw, telemetry, true);

        SamplingPipeline pipeline = new SamplingPipeline();
        pipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        pipeline.enable();

        // TODO make localizer find initial position (instead of relying on hardcoded values)
        drive.getLocalizer().setPoseEstimate(matchParameters.startingPosition.startingPosition);

        TrajectoryManager.update();

        msStuckDetectStop = 5000;
        while (!isStarted()) {
            if (isStopRequested()) {
                return;
            }
            drive.updatePoseEstimate();
            System.gc();
            telemetry.addData("Ready", "ready");
            telemetry.addData("Heading", radToDeg(drive.getExternalHeading()));
            telemetry.addData("Starting Position", matchParameters.startingPosition.startingPosition.toString());
            telemetry.addData("Sampling Pipeline Plurality", pipeline.plurality);
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }
        else {
            pipeline.disable();
        }
        Mineral position = pipeline.plurality;
        resetStartTime();

        if (position == Mineral.LEFT && matchParameters.startingPosition == StartingPosition.RED_FACING_DEPOT)
            Thread.sleep(5000);

        if (Math.abs(drive.getUnnormalizedHeading()) < 0.08) {
            drive.setOffset(matchParameters.startingPosition.heading);
        }

        LiftArmAction armAction = new LiftArmAction(0, 0.2, rw);
        Thread armActionThread = new Thread(armAction);
        armActionThread.start();
        if (!landed && opModeIsActive()) {
            land(rw, drivetrain);
        }


        Trajectory trajectory;
        TrajectoryBuilder builder;
        if (debugMineral) {
            matchParameters.mineralConfiguration = Mineral.values()[mineral];
            position = Mineral.values()[mineral];
        } else {
            matchParameters.mineralConfiguration = position;
        }

        builder = freshTrajectoryBuilder(drive);
        builder = builder.strafeRight(Math.sqrt(STRAFE_AMOUNT));
        trajectory = builder.build();
        drive.followTrajectory(trajectory);
        waitForTrajectoryFinish(drive, trajectory);

        builder = freshTrajectoryBuilder(drive);

        IntakeAction markerAction = null;


        boolean depositAtAndReverseEndFlag = false;
        boolean backServoFlag = false;
        boolean noExtendArmFlag = false;
        switch (matchParameters.startingPosition) {
            case RED_FACING_DEPOT:
                switch (matchParameters.mineralConfiguration) {
                    case LEFT:
                        if (matchParameters.parkOpponentCrater) {
                            builder = TrajectoryManager.load("red_depot_left", drive.getPoseEstimate());
                            markerAction = new IntakeAction(3, 5, rw);
                        }
                        else {
                            builder = builder.splineTo(new Pose2d(new Vector2d(46, -26), degToRad(-45))).splineTo(new Pose2d(new Vector2d(54.5, -46), degToRad(-90)));
                            builder = builder.splineTo(new Pose2d(new Vector2d(30, -63), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-23.5, -63), new ConstantInterpolator(degToRad(-180)));
                            markerAction = new IntakeAction(2.3, 4.5, rw);

                        }
                        backServoFlag = true;
                        noExtendArmFlag = true;
                        break;
                    case CENTER:
                        if (matchParameters.parkOpponentCrater) {
                            builder = TrajectoryManager.load("red_depot_center", drive.getPoseEstimate());
                            markerAction = new IntakeAction(4.8, 6.8, rw);
                        }
                        else {
                            builder = builder.turnTo(degToRad(-45)).splineTo(new Pose2d(new Vector2d(50, -57), degToRad(-90)))
                                    .splineTo(new Pose2d(new Vector2d(40, -62.5), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-16, -62.5));
                            markerAction = new IntakeAction(4, 6, rw);
                        }
                        break;
                    case RIGHT:

                        if (matchParameters.parkOpponentCrater) {
                            builder = TrajectoryManager.load("red_depot_right", drive.getPoseEstimate());
                            markerAction = new IntakeAction(6.5, 8.5, rw);
                            // builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -60), 0));
                            // builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-9, -60));
                            // markerAction = new IntakeAction(6.5, 8.5, rw);

                        }
                        else {
                            builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -60), 0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-9, -60));
                            markerAction = new IntakeAction(6.5, 8.5, rw);
                        }
                        break;
                }
                if (markerAction != null) {
                    Thread marker = new Thread(markerAction);
                    marker.start();
                }

                break;
            case RED_FACING_CRATER:
                depositAtAndReverseEndFlag = matchParameters.claim;
                noExtendArmFlag = matchParameters.claim;
                backServoFlag = true;

                SamplingArmAction samplingArmAction = null;

                switch (matchParameters.mineralConfiguration) {
                    case LEFT:
                        // builder = builder.turnTo(degToRad(-90)).lineTo(new Vector2d(-22, -44), new ConstantInterpolator(degToRad(-90))).turnTo(degToRad(-135));
                        if (matchParameters.claim) {
                            builder = TrajectoryManager.load("red_crater_left_claim_v2");
                            samplingArmAction = new SamplingArmAction(1.6, 3.25, rw);
                        }
                        else {
                            builder = TrajectoryManager.load("red_crater_left");
                        }
                        break;
                    case CENTER:
                        // builder = builder.turnTo(degToRad(-135)).lineTo(new Vector2d(-33.5, -33.5), new ConstantInterpolator(degToRad(-135)));
                        if (!matchParameters.claim) builder = TrajectoryManager.load("red_crater_center");
                        else {
                            builder = TrajectoryManager.load("red_crater_center_claim_v3");
                            samplingArmAction = new SamplingArmAction(2.5, 4, rw);
                        }
                        break;
                    case RIGHT:
                        // builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-44, -22), new ConstantInterpolator(degToRad(-180))).turnTo(degToRad(-135));
                        if (!matchParameters.claim) builder = TrajectoryManager.load("red_crater_right");
                        else {
                            builder = TrajectoryManager.load("red_crater_right_claim_v2");
                            samplingArmAction = new SamplingArmAction(1.5, 3.5, rw);
                        }
                        break;
                }
                if  (samplingArmAction != null) {
                    Thread arm = new Thread(samplingArmAction);
                    arm.start();
                }
                break;
            case BLUE_FACING_DEPOT:
                switch (matchParameters.mineralConfiguration) {
                    case LEFT:
                        // builder = builder.splineTo(new Pose2d(new Vector2d(-46, 26), degToRad(135))).splineTo(new Pose2d(new Vector2d(-54.5, 46), degToRad(90)));
                        // builder = builder.splineTo(new Pose2d(new Vector2d(-30, 63), degToRad(0))).turnTo(degToRad(0)).lineTo(new Vector2d(20, 63), new ConstantInterpolator(degToRad(0)));
                        builder = TrajectoryManager.load("red_depot_left", TrajectoryTransform.oneEighty());
                        markerAction = new IntakeAction(6.3, 8.5, rw);
                        break;
                    case CENTER:
                        builder = TrajectoryManager.load("red_depot_center", TrajectoryTransform.oneEighty());
                        markerAction = new IntakeAction(3.5, 5.5, rw);

                        break;
                    case RIGHT:
                        builder = TrajectoryManager.load("red_depot_right", TrajectoryTransform.oneEighty());
                        markerAction = new IntakeAction(6.5, 7.7, rw);
                        break;
                }
                if (markerAction != null) {
                    Thread marker = new Thread(markerAction);
                    marker.start();
                }
                break;
            case BLUE_FACING_CRATER:
                switch (matchParameters.mineralConfiguration) {
                    case LEFT:
                        // builder = builder.turnTo(degToRad(90)).lineTo(new Vector2d(22, 44), new ConstantInterpolator(degToRad(90))).turnTo(degToRad(45));
                        builder = TrajectoryManager.load("red_crater_left", TrajectoryTransform.oneEighty());
                        break;
                    case CENTER:
                        // builder = builder.turnTo(degToRad(45)).lineTo(new Vector2d(33.5, 33.5), new ConstantInterpolator(degToRad(45)));
                        builder = TrajectoryManager.load("red_crater_center", TrajectoryTransform.oneEighty());
                        break;
                    case RIGHT:
                        // builder = builder.turnTo(0).lineTo(new Vector2d(44, 22), new ConstantInterpolator(degToRad(0))).turnTo(degToRad(45));
                        builder = TrajectoryManager.load("red_crater_right", TrajectoryTransform.oneEighty());
                        break;
                }
                break;
        }

        trajectory = builder.build();
        drive.followTrajectory(trajectory);
        waitForTrajectoryFinish(drive, trajectory);

        // Stop drivetrain fully (could be residual power)
        drivetrain.stop();


        LinearSliderAction linearSliderAction = new LinearSliderAction(0, 1.5, rw);
        Thread linearSliderThread = new Thread(linearSliderAction);
        linearSliderThread.start();

        if (backServoFlag) {
            rw.backServo.setPosition(BACK_SERVO_DOWN);
        }

        if (depositAtAndReverseEndFlag) {
            markerAction = new IntakeAction(0, 2, rw);
            markerAction.setDelay(300);
            markerAction.start();
            Thread.sleep(1000);
            MoveQuick(-1, 2500, drivetrain, rw);
        }

        if (!noExtendArmFlag) {
            armController.setPositions(-1200, -1200);
            while (true) {
                if (isStopRequested()) return;
                telemetry.addData("Status", "Moving arm down");
                telemetry.update();
                armController.updateArmAuto(-1, 1);
            }
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.update();
        }
    }

    private TrajectoryBuilder freshTrajectoryBuilder(RoverRuckusMecanumDriveREVOptimized drive) {
        return new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
    }

    void waitForTrajectoryFinish(RoverRuckusMecanumDriveREVOptimized drive, Trajectory trajectory) {
        resetStartTime();
        double lastUpdate = getRuntime();
        if (debugPosition) {
            drive.drawSampledTrajectory(trajectory);
        }
        while (opModeIsActive() && drive.isFollowingTrajectory()) {
            drive.update();
            RobotLog.i(String.valueOf(getRuntime() - lastUpdate));
            lastUpdate = getRuntime();
            if (debugPosition) {
                drive.displayPosition();
            }

            telemetry.addData("Duration", trajectory.duration());
            telemetry.addData("Time elapsed", getRuntime());
            telemetry.update();
        }
    }

    private void MoveQuick(double speed, int counts, FourWheelMecanumDrivetrain drivetrain, RobotHardware hw) {
        int initialForward = hw.frontLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.setPowerAll(-1);

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

    private void AutoMove(double speed, double angle, int counts, FourWheelMecanumDrivetrain drivetrain, RobotHardware hw) {
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

    private void waitAndDisplayTrajectory(double time, FtcDashboard dashboard, TrajectoryBuilder trajectoryBuilder) {
        Trajectory trajectory = trajectoryBuilder.build();
        resetStartTime();
        while (getRuntime() < time) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void land(RobotHardware rw, FourWheelMecanumDrivetrain drivetrain) throws InterruptedException {
        rw.firstJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.firstJoint.setTargetPosition(-1100);
        rw.firstJoint.setPower(1);

        rw.backServo.setPosition(BACK_SERVO_UP);

        SamplingArmAction samplingArmAction = new SamplingArmAction(0, 2, rw);
        Thread thread = new Thread(samplingArmAction);
        thread.start();

        rw.pawServo.setPosition(LOCK_DISENGAGED);
        Thread.sleep(250);
        rw.linearSlider.setPower(-1);
        Thread.sleep(800);
        rw.linearSlider.setPower(1);
        Thread.sleep(TIME_GOING_DOWN);

        rw.firstJoint.setPower(0);
        rw.firstJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rw.linearSlider.setPower(0);
        Thread.sleep(250);
        rw.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.linearSlider.setTargetPosition(rw.linearSlider.getCurrentPosition() + 2200);
        rw.linearSlider.setPower(1);



        Thread.sleep(500);
        AutoMove(-0.5, 0, 250, drivetrain, rw);
        rw.linearSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rw.linearSlider.setPower(-1);
        Thread.sleep(1500);
        rw.linearSlider.setPower(0);
        AutoMove(0.5, 0, 250, drivetrain, rw);
    }

}
