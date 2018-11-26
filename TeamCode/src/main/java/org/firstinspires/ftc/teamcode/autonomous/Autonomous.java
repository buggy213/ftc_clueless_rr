package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous extends LinearOpMode {


    // TODO refine starting locations
    enum StartingPosition {
        RED_FACING_CRATER (new Pose2d(new Vector2d(-12, -12), -45)),
        RED_FACING_DEPOT (new Pose2d(new Vector2d(12, -12), 45)),
        BLUE_FACING_CRATER (new Pose2d(new Vector2d(12, 12), 135)),
        BLUE_FACING_DEPOT (new Pose2d(new Vector2d(-12, 12), 225));

        public final Pose2d startingPosition;
        StartingPosition(Pose2d startingPosition) {
            this.startingPosition = startingPosition;
        }
    }

    StartingPosition startingPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        RoverRuckusMecanumDriveREVOptimized drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            dumbStrafeLeft(0.5, 0.25, drive);

        }
    }

    void dumbStrafeLeft(double time, double power, RoverRuckusMecanumDriveREVOptimized drive) throws InterruptedException{
        drive.setMotorPowers(-power, power, -power, power);
        wait((long)(time / 1000));
        drive.setMotorPowers(0,0,0,0);
    }
}
