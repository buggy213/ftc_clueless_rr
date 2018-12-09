package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Drive;

import org.firstinspires.ftc.teamcode.autonomous.RoverRuckusMecanumDriveREVOptimized;

public class PositionBasedAction implements Action {

    double maxDistance;
    Vector2d startPos, endPos;
    RoverRuckusMecanumDriveREVOptimized drive;

    public PositionBasedAction(Vector2d startPos, Vector2d endPos, RoverRuckusMecanumDriveREVOptimized drive, double maxDistance) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.drive = drive;
        this.maxDistance = maxDistance;
    }
    @Override
    public void run() {
        watch();
    }

    @Override
    public void watch() {
        while (drive.getPoseEstimate().pos().distanceTo(startPos) > maxDistance) {

        }
        start();
    }

    @Override
    public void start() {
        update();
    }

    @Override
    public void update() {
        while(drive.getPoseEstimate().pos().distanceTo(endPos) > maxDistance) {

        }
        stop();
    }

    @Override
    public void stop() {

    }

    private Pose2d getTheoreticalPosition() {
        return drive.getPoseEstimate().minus(drive.getFollowingError());
    }
}
