package org.firstinspires.ftc.teamcode.autonomous.parameters;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public enum StartingPosition {
    RED_FACING_CRATER (new Pose2d(new Vector2d(-14.6, -14.6), 0), -Math.PI / 4),
    RED_FACING_DEPOT (new Pose2d(new Vector2d(14.6, -14.6), 0), Math.PI / 4),
    BLUE_FACING_CRATER (new Pose2d(new Vector2d(14.6, 14.6), 0), 5 * Math.PI / 4),
    BLUE_FACING_DEPOT (new Pose2d(new Vector2d(-14.6, 14.6), 0), 7 * Math.PI / 4);

    public final Pose2d startingPosition;
    public final double heading;
    StartingPosition(Pose2d startingPosition, double heading) {
        this.startingPosition = startingPosition;
        this.heading = heading;
    }
}