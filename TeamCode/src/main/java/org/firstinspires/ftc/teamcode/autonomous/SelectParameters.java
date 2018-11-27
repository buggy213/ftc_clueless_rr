package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SelectParameters extends LinearOpMode {
    public class Parameters {
        public StartingPosition startingPosition;

        @Override
        public String toString() {
            return startingPosition.name();
        }
    }

    public static Parameters matchParameters;

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

    @Override
    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            if (matchParameters == null) {
                matchParameters = new Parameters();
            }

            if (gamepad1.a) matchParameters.startingPosition = StartingPosition.RED_FACING_CRATER;
            if (gamepad1.b) matchParameters.startingPosition = StartingPosition.RED_FACING_DEPOT;
            if (gamepad1.x) matchParameters.startingPosition = StartingPosition.BLUE_FACING_CRATER;
            if (gamepad1.y) matchParameters.startingPosition = StartingPosition.BLUE_FACING_DEPOT;

            telemetry.addData("Starting Position", matchParameters);
        }
    }
}
