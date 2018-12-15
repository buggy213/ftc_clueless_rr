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

            telemetry.addData("Starting Position", matchParameters);
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
