package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Select Parameters")
public class SelectParameters extends LinearOpMode {
    public class Parameters {

        public StartingPosition startingPosition = StartingPosition.RED_FACING_DEPOT;
        //TODO vision
        public SamplingPosition mineralConfiguration = SamplingPosition.CENTER;

        @Override
        public String toString() {
            return startingPosition.name() + mineralConfiguration.name();
        }
    }
    public static Parameters matchParameters;


    // Position of gold mineral in sampling field
    enum SamplingPosition {
        LEFT, CENTER, RIGHT
    }


    // TODO refine starting locations
    enum StartingPosition {
        RED_FACING_CRATER (new Pose2d(new Vector2d(-12, -12), 0)),
        RED_FACING_DEPOT (new Pose2d(new Vector2d(12, -12), 0)),
        BLUE_FACING_CRATER (new Pose2d(new Vector2d(12, 12), 0)),
        BLUE_FACING_DEPOT (new Pose2d(new Vector2d(-12, 12), 0));

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
            telemetry.update();
        }
    }

    public Parameters initalize(){
        if (matchParameters == null) {
            matchParameters = new Parameters();
        }

        return matchParameters;
    }
}
