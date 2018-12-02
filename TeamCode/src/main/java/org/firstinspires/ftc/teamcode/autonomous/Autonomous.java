package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Localizer;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;
import org.firstinspires.ftc.teamcode.motionplanningtest.drive.ComplementaryVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.motionplanningtest.drive.DriveConstants;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Main Autonomous")
public class Autonomous extends LinearOpMode {

    VuforiaLocalizer vuforia;

    final double SAMPLING_SERVO_UP = 1;
    final double SAMPLING_SERVO_DOWN = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        RoverRuckusMecanumDriveREVOptimized drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap, degToRad(315));
        RobotHardware rw = new RobotHardware(hardwareMap);
        SelectParameters.Parameters matchParameters = SelectParameters.matchParameters;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AcJSEaj/////AAABmZmYKDlSrk/tqCYXQbWF2zIZKePATp9JlOVeqyDFYAOLOJcwm5eO/EyXGAUCjoeJ2LwBWY5JdtBWhaRt3GVVLchqkKSMg8nrFAyAoQUAhLx1i+/9pcqq0Z7HMByQMzWK216ENi7sQ2GrKnETBbbt+82fyitozuTSFr1LAbIsbDd4IjsTmK7CRBhG0Ns9cOr7+prlgNiGHKYhvRwfvV8asHK76mCTPInkYhHMoC8bm+00Z3vzlFhNrDjOu4LpjVXH8KD2ksauFeM1lt3OOLqebuz4IbYkxTN33ZPU1pqJb9qYFaWIMO/7q6XTXMr9fO0Jm0zFt6N+yCe4lWq1UHrnG/MiK8CS30m/eLxItELpmHj7";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // ComplementaryVuforiaLocalizer localizer = new ComplementaryVuforiaLocalizer(vuforia, VuforiaLocalizer.CameraDirection.BACK, drive.getLocalizer());

        // TODO make localizer find initial position (instead of relying on hardcoded values)
        drive.getLocalizer().setPoseEstimate(matchParameters.startingPosition.startingPosition);

        telemetry.addData("Ready", "ready");
        telemetry.update();
        waitForStart();

        Trajectory trajectory;
        TrajectoryBuilderExtended builder;
        while (opModeIsActive()) {

            builder = freshTrajectoryBuilder(drive);
            switch (matchParameters.mineralConfiguration) {
                case LEFT:
                    builder = builder.movePolarRelative(32, -45);
                    break;
                case CENTER:
                    builder = builder.strafeRight(30);
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

            /*builder = freshTrajectoryBuilder(drive);
            switch (matchParameters.mineralConfiguration) {
                case LEFT:
                    builder = builder.back(10);
                    break;
                case CENTER:
                    builder = builder.forward(10);
                    break;
                case RIGHT:
                    builder = builder.forward(10);
                    break;
            }
            trajectory = builder.build();

            drive.followTrajectory(trajectory);

            waitForTrajectoryFinish(drive);

            rw.samplingServo.setPosition(SAMPLING_SERVO_UP);

            return;

            /*builder = freshTrajectoryBuilder(drive);

            switch (matchParameters.startingPosition) {
                case RED_FACING_DEPOT:
                    builder.lineTo(new Vector2d(60, 0), new ConstantInterpolator(3/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(60, 48), 1/2 * Math.PI));
                    break;
                case RED_FACING_CRATER:
                    builder.lineTo(new Vector2d(60, 0), new ConstantInterpolator(3/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(60, 48), 1/2 * Math.PI));
                    break;
                case BLUE_FACING_DEPOT:
                    builder.lineTo(new Vector2d(-60, 0), new ConstantInterpolator(7/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(-60, -48), 3/2 * Math.PI));
                    break;
                case BLUE_FACING_CRATER:
                    builder.lineTo(new Vector2d(-60, 0), new ConstantInterpolator(5/4 * Math.PI));
                    builder.splineTo(new Pose2d(new Vector2d(-60, -48), 3/2 * Math.PI));
                    break;
            }

            trajectory = builder.build();
            drive.followTrajectory(trajectory);
            waitForTrajectoryFinish(drive);

            trajectory = freshTrajectoryBuilder(drive).back(72).build(); */

            requestOpModeStop();
        }
    }

    TrajectoryBuilderExtended freshTrajectoryBuilder(RoverRuckusMecanumDriveREVOptimized drive) {
        return new TrajectoryBuilderExtended(drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
    }

    void waitForTrajectoryFinish(RoverRuckusMecanumDriveREVOptimized drive) {
        while (opModeIsActive() && drive.isFollowingTrajectory()) {
            drive.update();
        }
    }

    void dumbStrafeLeft(double time, double power, RoverRuckusMecanumDriveREVOptimized drive) throws InterruptedException{
        drive.setMotorPowers(-power, power, -power, power);
        wait((long)(time / 1000));
        drive.setMotorPowers(0,0,0,0);
    }

    double degToRad(double deg) {
        return Math.PI * deg / 180;
    }

    double radToDeg(double rad) {
        return rad * 180 / Math.PI;
    }


}
