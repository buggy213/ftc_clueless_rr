package org.firstinspires.ftc.teamcode.motionplanningtest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@Autonomous
public class MotorTest extends LinearOpMode {
    public static String MOTOR_NAME = "forwardLeft";
    DcMotor motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.dcMotor.get(MOTOR_NAME);
        waitForStart();
        while(true) {
            motor.setPower(0.5);
        }
    }
}
