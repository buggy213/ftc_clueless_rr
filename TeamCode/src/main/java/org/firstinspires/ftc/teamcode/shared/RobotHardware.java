package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
        public DcMotorEx backLeft, backRight, frontLeft, frontRight, linearSlider, firstJoint, secondJoint, intakeJoint;

        public Servo backServo, pawServo, doorServo, samplingServo;
        public CRServo intake;
        public MotorServo firstJointVirtualServo, secondJointVirtualServo;

        public HardwareMap hardwareMap;

    public RobotHardware(HardwareMap hwMap) {
            backLeft = (DcMotorEx)hwMap.get(DcMotor.class, "backLeft");
            backRight = (DcMotorEx)hwMap.get(DcMotor.class, "backRight");
            frontLeft = (DcMotorEx)hwMap.get(DcMotor.class,"frontLeft");
            frontRight = (DcMotorEx)hwMap.get(DcMotor.class,"frontRight");
            linearSlider = (DcMotorEx)hwMap.get(DcMotor.class, "linearSlider");
            firstJoint = (DcMotorEx)hwMap.get(DcMotor.class, "firstJoint");
            secondJoint = (DcMotorEx)hwMap.get(DcMotor.class, "secondJoint");

            backServo = hwMap.get(Servo.class, "backServo");
            samplingServo = hwMap.get(Servo.class, "samplingServo");

            intakeJoint = (DcMotorEx)hwMap.get(DcMotor.class, "intakeJoint");
            intake = hwMap.get(CRServo.class, "intake");
            pawServo = hwMap.get(Servo.class, "pawServo");
            doorServo = hwMap.get(Servo.class, "doorServo");

            firstJointVirtualServo = new MotorServo(firstJoint, MotorServo.MotorConfiguration.firstJoint);
            secondJointVirtualServo = new MotorServo(secondJoint, MotorServo.MotorConfiguration.secondJoint);

            this.hardwareMap = hwMap;
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