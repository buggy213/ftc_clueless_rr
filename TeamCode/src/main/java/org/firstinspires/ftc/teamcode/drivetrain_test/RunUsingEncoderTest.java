package org.firstinspires.ftc.teamcode.drivetrain_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Run Using Encoder Test")
public class RunUsingEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware rw = new RobotHardware(hardwareMap);
        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(rw);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain.resetEncoders();

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rw.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rw.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rw.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rw.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        ElapsedTime time = new ElapsedTime();

        drivetrain.setPowerAll(0.08);
        while (time.milliseconds() < 1000) {
            drivetrain.displayInformation();
        }
        drivetrain.setPowerAll(0);
    }
}
