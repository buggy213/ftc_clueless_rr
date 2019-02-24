package org.firstinspires.ftc.teamcode.autonomous.parameters;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Select Parameters")
public class SelectParameters extends LinearOpMode {

    public static Parameters matchParameters;

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
            if (gamepad1.left_bumper) matchParameters.parkOpponentCrater = true;
            if (gamepad1.right_bumper) matchParameters.parkOpponentCrater = false;
            if (gamepad1.dpad_left) matchParameters.claim = true;
            if (gamepad1.dpad_right) matchParameters.claim = false;
            telemetry.addData("Starting Position", matchParameters.startingPosition);
            telemetry.addData("Parking at opponent's crater: ", matchParameters.parkOpponentCrater);
            telemetry.update();
        }
    }

    public static Parameters getMatchParameters(){
        if (matchParameters == null) {
            matchParameters = new Parameters();
        }

        return matchParameters;
    }
}
