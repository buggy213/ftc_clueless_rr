package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Localizer;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.motionplanningtest.drive.ComplementaryVuforiaLocalizer;

public class Autonomous extends LinearOpMode {

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        RoverRuckusMecanumDriveREVOptimized drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap);

        SelectParameters.Parameters matchParameters = SelectParameters.matchParameters;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AcJSEaj/////AAABmZmYKDlSrk/tqCYXQbWF2zIZKePATp9JlOVeqyDFYAOLOJcwm5eO/EyXGAUCjoeJ2LwBWY5JdtBWhaRt3GVVLchqkKSMg8nrFAyAoQUAhLx1i+/9pcqq0Z7HMByQMzWK216ENi7sQ2GrKnETBbbt+82fyitozuTSFr1LAbIsbDd4IjsTmK7CRBhG0Ns9cOr7+prlgNiGHKYhvRwfvV8asHK76mCTPInkYhHMoC8bm+00Z3vzlFhNrDjOu4LpjVXH8KD2ksauFeM1lt3OOLqebuz4IbYkxTN33ZPU1pqJb9qYFaWIMO/7q6XTXMr9fO0Jm0zFt6N+yCe4lWq1UHrnG/MiK8CS30m/eLxItELpmHj7";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // ComplementaryVuforiaLocalizer localizer = new ComplementaryVuforiaLocalizer(vuforia, VuforiaLocalizer.CameraDirection.BACK, drive.getLocalizer());

        // TODO make localizer find initial position (instead of relying on hardcoded values)
        drive.getLocalizer().setPoseEstimate(matchParameters.startingPosition.startingPosition);

        waitForStart();

        while (opModeIsActive()) {

        }
    }

    void dumbStrafeLeft(double time, double power, RoverRuckusMecanumDriveREVOptimized drive) throws InterruptedException{
        drive.setMotorPowers(-power, power, -power, power);
        wait((long)(time / 1000));
        drive.setMotorPowers(0,0,0,0);
    }
}
