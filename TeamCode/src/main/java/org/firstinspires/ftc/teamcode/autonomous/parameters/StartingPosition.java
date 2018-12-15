package org.firstinspires.ftc.teamcode.autonomous.parameters;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public enum StartingPosition {
    RED_FACING_CRATER (new Pose2d(new Vector2d(-12, -12), 0)),
    RED_FACING_DEPOT (new Pose2d(new Vector2d(12, -12), 0)),
    BLUE_FACING_CRATER (new Pose2d(new Vector2d(12, 12), 0)),
    BLUE_FACING_DEPOT (new Pose2d(new Vector2d(-12, 12), 0));

    public final Pose2d startingPosition;
    StartingPosition(Pose2d startingPosition) {
        this.startingPosition = startingPosition;
    }
}