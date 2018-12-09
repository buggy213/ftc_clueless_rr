package org.firstinspires.ftc.teamcode.armkinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;

import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.MANUAL_SPEED;

@Config
public class ArmController {
    RobotHardware robotHardware;

    // region constants
    static final double FIRST_JOINT_LENGTH = 18;
    static final double SECOND_JOINT_LENGTH = 17.5;

    static final double FIRST_JOINT_GEAR_RATIO = .333333;
    static final double SECOND_JOINT_GEAR_RATIO = .666666;

    // Constant which translates encoder ticks to real world rotations (assuming Neverest 60s are used)
    static final double FIRST_JOINT_ENCODER_RATIO = 1680 / FIRST_JOINT_GEAR_RATIO;
    static final double SECOND_JOINT_ENCODER_RATIO = 1680 / SECOND_JOINT_GEAR_RATIO;

    static final double rotationFactor = 0.0005;
    static final double horizontalFactor = 0.0025;



    static final double MAX_SPEED = 1;
    // endregion

    TwoJointedArmKinematics kinematics;

    Telemetry telemetry;

    // Cartesian
    double endEffectorPositionX = 35.5;
    double endEffectorPositionY = 0;

    // Polar
    double length = 35.4;
    double angle = 0;

    double firstJointTarget = 0;
    double secondJointTarget = 0;

    public static boolean enabled = true;

    public static double firstJointOffset;
    public static double secondJointOffset;

    PIDController firstJointPID;
    PIDController secondJointPID;

    public void setEnabled(boolean enabled) {
        ArmController.enabled = enabled;
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public ArmController(RobotHardware robotHardware, Telemetry telemetry, boolean enabled) {
        this.robotHardware = robotHardware;
        kinematics = new TwoJointedArmKinematics(FIRST_JOINT_LENGTH, SECOND_JOINT_LENGTH);
        robotHardware.firstJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.secondJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setEnabled(enabled);
        robotHardware.firstJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.secondJoint.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;

        firstJointPID = new PIDController(RobotConstants.FIRST_JOINT_PID);
        secondJointPID = new PIDController(RobotConstants.SECOND_JOINT_PID);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        robotHardware.firstJoint.setMode(runMode);
        robotHardware.secondJoint.setMode(runMode);
    }

    public void updateArmTeleop(Gamepad playerB, double dt) {
        double horizontal = playerB.left_stick_y * horizontalFactor;
        double rotation = (playerB.right_stick_y) * rotationFactor;

        length += horizontal;
        // don't want length to exactly equal max
        length = Range.clip(length, 0, FIRST_JOINT_LENGTH + SECOND_JOINT_LENGTH - 0.01);

        angle += rotation;
        angle = Range.clip(angle, -Math.PI, 0);

        endEffectorPositionX = length * Math.cos(angle);
        endEffectorPositionY = length * Math.sin(angle);
        double[] motorAngles;
        double firstJointError;
        double secondJointError;

        if (!(rotation == 0 && horizontal == 0)) {
            motorAngles = kinematics.inverseKinematics(endEffectorPositionX, endEffectorPositionY);
            double firstMotorAngle = endEffectorPositionX > 0 ? motorAngles[0] : motorAngles[2];
            double secondMotorAngle = endEffectorPositionX > 0 ? motorAngles[1] : motorAngles[3];
            firstJointTarget = (int)(firstMotorAngle * (1 / (2 * Math.PI)) * FIRST_JOINT_ENCODER_RATIO);
            secondJointTarget = (int)(secondMotorAngle * (1 / (2 * Math.PI)) * SECOND_JOINT_ENCODER_RATIO);
        }

        firstJointError = firstJointTarget - robotHardware.firstJoint.getCurrentPosition();
        secondJointError = secondJointTarget - robotHardware.secondJoint.getCurrentPosition();

        if (enabled) {
            robotHardware.firstJoint.setPower(firstJointPID.feedback(firstJointError));
            robotHardware.secondJoint.setPower(secondJointPID.feedback(secondJointError));
        }

        telemetry.addData("horizontal", horizontal);
        telemetry.addData("rotation", rotation);
        telemetry.addData("length", length);
        telemetry.addData("angle", angle);
        telemetry.addData("x", endEffectorPositionX);
        telemetry.addData("y", endEffectorPositionY);
        telemetry.addData("first joint target", firstJointTarget);
        telemetry.addData("second joint target", secondJointTarget);
        telemetry.addData("first joint error", firstJointError);
        telemetry.addData("second joint error", secondJointError);
    }

    public void manualArmControl(Gamepad gamepad) {
        firstJointTarget += gamepad.left_stick_y * MANUAL_SPEED;
        secondJointTarget += (gamepad.right_stick_y) * MANUAL_SPEED;

        double firstJointError = firstJointTarget - robotHardware.firstJoint.getCurrentPosition();
        double secondJointError = secondJointTarget - robotHardware.secondJoint.getCurrentPosition();

        robotHardware.firstJoint.setPower(firstJointPID.feedback(firstJointError));
        robotHardware.secondJoint.setPower(secondJointPID.feedback(secondJointError));


        telemetry.addData("first joint target", firstJointTarget);
        telemetry.addData("second joint target", secondJointTarget);
        telemetry.addData("first joint error", firstJointError);
        telemetry.addData("second joint error", secondJointError);
    }

    public void setPositions(int firstJointTarget, int secondJointTarget) {
        this.firstJointTarget = firstJointTarget;
        this.secondJointTarget = secondJointTarget;
    }

    public void updateArmAuto() {
        double firstJointError = firstJointTarget - robotHardware.firstJoint.getCurrentPosition();
        double secondJointError = secondJointTarget - robotHardware.secondJoint.getCurrentPosition();

        robotHardware.firstJoint.setPower(firstJointPID.feedback(firstJointError));
        robotHardware.secondJoint.setPower(secondJointPID.feedback(secondJointError));
    }
}
