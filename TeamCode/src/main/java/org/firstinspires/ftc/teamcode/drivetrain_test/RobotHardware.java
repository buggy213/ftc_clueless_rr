package org.firstinspires.ftc.teamcode.drivetrain_test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MotorServo;

public class RobotHardware {
        public DcMotorEx backLeft, backRight, frontLeft, frontRight, linearSlider, firstJoint, secondJoint;

        public Servo samplingServo;
        public MotorServo firstJointVirtualServo, secondJointVirtualServo;

        public RobotHardware(HardwareMap hwMap) {
            backLeft = (DcMotorEx)hwMap.get(DcMotor.class, "backLeft");
            backRight = (DcMotorEx)hwMap.get(DcMotor.class, "backRight");
            frontLeft = (DcMotorEx)hwMap.get(DcMotor.class,"frontLeft");
            frontRight = (DcMotorEx)hwMap.get(DcMotor.class,"frontRight");
            linearSlider = (DcMotorEx)hwMap.get(DcMotor.class, "linearSlider");
            firstJoint = (DcMotorEx)hwMap.get(DcMotor.class, "firstJoint");
            secondJoint = (DcMotorEx)hwMap.get(DcMotor.class, "secondJoint");

            samplingServo = hwMap.get(Servo.class, "samplingServo");

            firstJointVirtualServo = new MotorServo(firstJoint, MotorServo.MotorConfiguration.firstJoint);
            secondJointVirtualServo = new MotorServo(secondJoint, MotorServo.MotorConfiguration.secondJoint);


        }

        public void resetEncoders() throws InterruptedException{
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Thread.sleep(150);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }