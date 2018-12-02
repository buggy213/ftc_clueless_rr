package org.firstinspires.ftc.teamcode.armkinematics;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;

public class ArmController {
    RobotHardware robotHardware;

    static final double FIRST_JOINT_LENGTH = 18;
    static final double SECOND_JOINT_LENGTH = 17.5;

    static final double FIRST_JOINT_GEAR_RATIO = .333333;
    static final double SECOND_JOINT_GEAR_RATIO = .666666;

    // Constant which translates encoder ticks to real world rotations (assuming Neverest 60s are used)
    static final double FIRST_JOINT_ENCODER_RATIO = 1680 / FIRST_JOINT_GEAR_RATIO;
    static final double SECOND_JOINT_ENCODER_RATIO = 1680 / SECOND_JOINT_GEAR_RATIO;

    static final double rotationFactor = 0.075;
    static final double horizontalFactor = 1;

    static final double MAX_SPEED = 1;

    TwoJointedArmKinematics kinematics;

    Telemetry telemetry;

    // Cartesian
    double endEffectorPositionX = 35.5;
    double endEffectorPositionY = 0;

    // Polar
    double length = 35.5;
    double angle = 0;

    int firstJointTarget = 0;
    int secondJointTarget = 0;

    public ArmController(RobotHardware robotHardware, Telemetry telemetry) {
        this.robotHardware = robotHardware;
        kinematics = new TwoJointedArmKinematics(FIRST_JOINT_LENGTH, SECOND_JOINT_LENGTH);
        robotHardware.firstJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.secondJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.firstJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.secondJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.telemetry = telemetry;
    }

    public void updateArm(Gamepad playerB, double dt) {
        double horizontal = playerB.left_stick_x * dt * horizontalFactor;
        double rotation = (playerB.right_trigger - playerB.left_trigger) * dt * rotationFactor;

        length += horizontal;
        length = Range.clip(length, 0, FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH);

        angle += rotation;
        angle = Range.clip(angle, 0, Math.PI);

        endEffectorPositionX = length * Math.cos(angle);
        endEffectorPositionY = length * Math.sin(angle);
        double[] motorAngles;


        if (!(rotation == 0 && horizontal == 0)) {
            motorAngles = kinematics.inverseKinematics(endEffectorPositionX, endEffectorPositionY);
            firstJointTarget = (int)(motorAngles[0] * FIRST_JOINT_ENCODER_RATIO);
            secondJointTarget = (int)(motorAngles[1] * SECOND_JOINT_ENCODER_RATIO);

            robotHardware.firstJoint.setTargetPosition(firstJointTarget);
            robotHardware.secondJoint.setTargetPosition(secondJointTarget);
        }

        telemetry.addData("horizontal", horizontal);
        telemetry.addData("rotation", rotation);
        telemetry.addData("length", length);
        telemetry.addData("angle", angle);
        telemetry.addData("x", endEffectorPositionX);
        telemetry.addData("y", endEffectorPositionY);
        telemetry.addData("first joint target", firstJointTarget);
        telemetry.addData("second joint target", secondJointTarget);

        int firstJointError = firstJointTarget - robotHardware.firstJoint.getCurrentPosition();
        int secondJointError = secondJointTarget - robotHardware.secondJoint.getCurrentPosition();


        telemetry.addData("first joint error", firstJointError);
        telemetry.addData("second joint error", secondJointError);
        boolean retFlag = false;
        if (firstJointError == 0 && secondJointError == 0) {
            robotHardware.firstJoint.setPower(0);
            robotHardware.secondJoint.setPower(0);
            retFlag = true;
        }
        else if (firstJointError == 0) {
            robotHardware.secondJoint.setPower(MAX_SPEED);
            robotHardware.firstJoint.setPower(0);
            retFlag = true;
        }
        else if (secondJointError == 0) {
            robotHardware.secondJoint.setPower(0);
            robotHardware.firstJoint.setPower(MAX_SPEED);
            retFlag = true;
        }



        if (retFlag) return;

        if (firstJointError > secondJointError) {
            robotHardware.secondJoint.setPower(MAX_SPEED);
            robotHardware.firstJoint.setPower(MAX_SPEED / (firstJointError / secondJointError));
        }
        else {
            robotHardware.firstJoint.setPower(MAX_SPEED);
            robotHardware.secondJoint.setPower(MAX_SPEED / (secondJointError / firstJointError));
        }

    }
}
