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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.parameters.Mineral;
import org.firstinspires.ftc.teamcode.autonomous.parameters.Parameters;
import org.firstinspires.ftc.teamcode.autonomous.parameters.SelectParameters;
import org.firstinspires.ftc.teamcode.autonomous.vision.SamplingPipeline;
import org.firstinspires.ftc.teamcode.motionplanning.drive.config.DriveConstants;
import org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize.TrajectoryManager;
import org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize.TrajectoryTransform;
import org.firstinspires.ftc.teamcode.motionplanning.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;
import org.firstinspires.ftc.teamcode.shared.RoverRuckusMecanumDriveREVOptimized;

import java.util.List;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_UP;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.LOCK_DISENGAGED;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Main Autonomous")
@Config
public class Autonomous extends LinearOpMode {

    public static boolean debug = false;
    public static boolean landed = false;
    public static int mineral = 0;

    public static int TIME_GOING_DOWN = 2000;

    public static double STRAFE_AMOUNT = 200;

    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    private static final String VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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
        Mineral position = pipeline.plurality;

        if (Math.abs(drive.getUnnormalizedHeading()) < 0.08) {
            drive.setOffset(matchParameters.startingPosition.heading);
        }

        if (!landed) {
            land(rw, drivetrain);
        }

        Trajectory trajectory;
        TrajectoryBuilder builder;
        if (debug) {
            matchParameters.mineralConfiguration = Mineral.values()[mineral];
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
        switch (matchParameters.startingPosition) {
            case RED_FACING_DEPOT:

                switch (matchParameters.mineralConfiguration) {
                    case LEFT:
                        if (matchParameters.parkOpponentCrater) {
                            builder = TrajectoryManager.load("red_depot_left", drive.getPoseEstimate());
                            markerAction = new IntakeAction(4.3, 5.5, rw);
                        }
                        else {
                            builder = builder.splineTo(new Pose2d(new Vector2d(46, -26), degToRad(-45))).splineTo(new Pose2d(new Vector2d(54.5, -46), degToRad(-90)));
                            builder = builder.splineTo(new Pose2d(new Vector2d(30, -63), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-20, -63), new ConstantInterpolator(degToRad(-180)));
                            markerAction = new IntakeAction(4.3, 5.5, rw);
                        }
                        break;
                    case CENTER:
                        if (matchParameters.parkOpponentCrater) {
                            builder = TrajectoryManager.load("red_depot_center", drive.getPoseEstimate());
                            markerAction = new IntakeAction(6, 8, rw);
                        }
                        else {
                            builder = builder.turnTo(degToRad(-45)).splineTo(new Pose2d(new Vector2d(50, -57), degToRad(-90)))
                                    .splineTo(new Pose2d(new Vector2d(40, -62.5), degToRad(-180))).turnTo(degToRad(-180)).lineTo(new Vector2d(-16, -62.5));
                            markerAction = new IntakeAction(6, 8, rw);
                        }
                        break;
                    case RIGHT:
                        if (matchParameters.parkOpponentCrater) {
                            builder = TrajectoryManager.load("red_depot_right", drive.getPoseEstimate());
                            markerAction = new IntakeAction(9, 11, rw);
                        }
                        else {
                            builder = builder.turnTo(degToRad(-90)).splineTo(new Pose2d(new Vector2d(48, -60), 0));
                            builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-9, -60));
                            markerAction = new IntakeAction(7, 9, rw);
                        }
                        break;
                }
                if (markerAction != null) {
                    Thread marker = new Thread(markerAction);
                    marker.start();
                }

                break;
            case RED_FACING_CRATER:
                switch (matchParameters.mineralConfiguration) {
                    case LEFT:
                        // builder = builder.turnTo(degToRad(-90)).lineTo(new Vector2d(-22, -44), new ConstantInterpolator(degToRad(-90))).turnTo(degToRad(-135));
                        builder = TrajectoryManager.load("red_crater_left");
                        break;
                    case CENTER:
                        // builder = builder.turnTo(degToRad(-135)).lineTo(new Vector2d(-33.5, -33.5), new ConstantInterpolator(degToRad(-135)));
                        builder = TrajectoryManager.load("red_crater_center");
                        break;
                    case RIGHT:
                        // builder = builder.turnTo(degToRad(-180)).lineTo(new Vector2d(-44, -22), new ConstantInterpolator(degToRad(-180))).turnTo(degToRad(-135));
                        builder = TrajectoryManager.load("red_crater_right");
                        break;
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
                        markerAction = new IntakeAction(4.3, 5.5, rw);

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

        armController.setPositions(-1050, -1200);
        while (opModeIsActive()) {
            armController.updateArmAuto(-0.25, 0.25);
        }
    }

    private TrajectoryBuilder freshTrajectoryBuilder(RoverRuckusMecanumDriveREVOptimized drive) {
        return new TrajectoryBuilder(drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
    }

    void waitForTrajectoryFinish(RoverRuckusMecanumDriveREVOptimized drive, Trajectory trajectory) {
        double lastUpdate = getRuntime();
        if (debug) {
            drive.drawSampledTrajectory(trajectory);
        }
        while (opModeIsActive() && drive.isFollowingTrajectory()) {
            drive.update();
            RobotLog.i(String.valueOf(getRuntime() - lastUpdate));
            lastUpdate = getRuntime();
            if (debug) {
                drive.displayPosition();
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

    public void tfodDetect() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                        }
                    }
                }
                telemetry.update();
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void land(RobotHardware rw, FourWheelMecanumDrivetrain drivetrain) throws InterruptedException {

        rw.pawServo.setPosition(LOCK_DISENGAGED);
        Thread.sleep(250);
        rw.linearSlider.setPower(-1);
        Thread.sleep(800);
        rw.linearSlider.setPower(0.45);
        Thread.sleep(TIME_GOING_DOWN);
        rw.linearSlider.setPower(0);
        Thread.sleep(250);
        rw.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.linearSlider.setTargetPosition(rw.linearSlider.getCurrentPosition() + 2400);
        rw.linearSlider.setPower(1);
        Thread.sleep(650);
        AutoMove(-0.25, 0, 250, drivetrain, rw);
        rw.linearSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rw.linearSlider.setPower(-1);
        Thread.sleep(1500);
        rw.linearSlider.setPower(0);
        AutoMove(0.25, 0, 250, drivetrain, rw);
    }

}
