package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.armkinematics.PIDController;
import org.firstinspires.ftc.teamcode.arm.armkinematics.TwoJointedArmKinematics;
import org.firstinspires.ftc.teamcode.shared.RobotConstants;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.TelemetryOpmode;

import java.io.FileWriter;
import java.io.IOException;

@Config
public class ArmController {
    RobotHardware robotHardware;

    // region constants
    static final double FIRST_JOINT_LENGTH = 18;
    static final double SECOND_JOINT_LENGTH = 17.5;


    // Prevent bouncing
    static final int MAX_CORRECT_AMOUNT = 30;

    // Constant which translates encoder ticks to real world rotations
    static final double FIRST_JOINT_ENCODER_RATIO = 7168;
    static final double SECOND_JOINT_ENCODER_RATIO = 6720;

    static final double rotationFactor = 0.0005;
    static final double horizontalFactor = 0.0025;

    static final double MAX_SPEED = 0.5;

    static final int TARGET_TOLERANCE = 50;
    // endregion

    TwoJointedArmKinematics kinematics;

    Telemetry telemetry;

    // Cartesian
    double endEffectorPositionX = 35.5;
    double endEffectorPositionY = 0;

    // Polar
    double length = 35.4;
    double angle = 0;

    static final double MANUAL_SPEED = 0.6;

    double firstJointTarget = 0;
    double secondJointTarget = 0;

    public static boolean enabled = true;

    public static double firstJointOffset;
    public static double secondJointOffset;

    PIDController firstJointPID;
    PIDController secondJointPID;

    Gamepad previousGamepad;

    public void setEnabled(boolean enabled) {
        ArmController.enabled = enabled;
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        secondJointCompensate = new PIDFController(new PIDCoefficients(0.02, 0, 0));
        secondJointCompensate.setTargetPosition(0);

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

    double initialFirstJoint, initialSecondJoint = 0;
    double kA = 500; // -1 to 1 to angular velocity (ticks/s)
    double ratio = -2; // Ratio between first joint angular velocity and second joint angular velocity
    PIDFController secondJointCompensate;
    // Only implements kinematic control for lengthwise movement
    public void basicKinematicControl (Gamepad gamepad) {
        if (previousGamepad == null) {
            previousGamepad = new Gamepad();
            try {
                previousGamepad.copy(gamepad);
            }
            catch (RobotCoreException e) {
                RobotLog.e(e.toString());
            }
        }
        double firstJointError = firstJointTarget - robotHardware.firstJoint.getCurrentPosition();
        double secondJointError = secondJointTarget - robotHardware.secondJoint.getCurrentPosition();

        double firstJointPower;
        double secondJointPower;

        if (Math.abs(firstJointError) > MAX_CORRECT_AMOUNT) {
            // Reset
            firstJointTarget = robotHardware.firstJoint.getCurrentPosition();
        }
        if (Math.abs(secondJointError) > MAX_CORRECT_AMOUNT) {
            secondJointTarget = robotHardware.secondJoint.getCurrentPosition();
        }




        telemetry.addData("first joint target", firstJointTarget);
        telemetry.addData("second joint target", secondJointTarget);
        telemetry.addData("first joint error", firstJointError);
        telemetry.addData("second joint error", secondJointError);

        if (gamepad.left_stick_y == 0 && previousGamepad.left_stick_y != 0 && gamepad.right_stick_y == 0 && previousGamepad.right_stick_y != 0) {
            firstJointTarget = robotHardware.firstJoint.getCurrentPosition();
            firstJointPID.reset();
            secondJointTarget = robotHardware.secondJoint.getCurrentPosition();
            secondJointPID.reset();
        }
        else if (gamepad.left_stick_y == 0 && gamepad.right_stick_y == 0) {
            firstJointPower = firstJointPID.feedback(firstJointError);
            secondJointPower = secondJointPID.feedback(secondJointError);
            robotHardware.firstJoint.setPower(firstJointPower);
            robotHardware.secondJoint.setPower(secondJointPower);
        }
        else if (gamepad.left_stick_y == 0) {
            robotHardware.firstJoint.setPower(gamepad.left_stick_y);
        }

        if (gamepad.right_stick_y != 0) {
            if (initialFirstJoint == 0 || initialSecondJoint == 0) {
                initialFirstJoint = robotHardware.firstJoint.getCurrentPosition();
                initialSecondJoint = robotHardware.secondJoint.getCurrentPosition();
            }
            robotHardware.firstJoint.setVelocity(kA * gamepad.right_stick_y);
            // Compensate with second joint (easier?)
            double target = Math.abs(ratio) * (robotHardware.firstJoint.getCurrentPosition() - initialFirstJoint);
            double current = robotHardware.secondJoint.getCurrentPosition() - initialSecondJoint;
            double feedback = secondJointCompensate.update(target - current) * kA;

            telemetry.addData("Feedback", feedback);
            telemetry.addData("Target", target);
            telemetry.addData("Current", current);

            robotHardware.secondJoint.setVelocity(-ratio * kA * gamepad.right_stick_y - feedback);


        }
        else {
            initialFirstJoint = 0;
            initialSecondJoint = 0;
        }

        try {
            previousGamepad.copy(gamepad);
        }
        catch (RobotCoreException e) {
            RobotLog.e(e.toString());
        }
    }

    public void manualArmControl(Gamepad gamepad) {

        if (previousGamepad == null) {
            previousGamepad = new Gamepad();
            try {
                previousGamepad.copy(gamepad);
            }
            catch (RobotCoreException e) {
                RobotLog.e(e.toString());
            }
        }

        double firstJointError = firstJointTarget - robotHardware.firstJoint.getCurrentPosition();
        double secondJointError = secondJointTarget - robotHardware.secondJoint.getCurrentPosition();

        double firstJointPower;
        double secondJointPower;



        if (gamepad.left_stick_y == 0 && previousGamepad.left_stick_y != 0) {
            firstJointTarget = robotHardware.firstJoint.getCurrentPosition();
            firstJointPID.reset();
            firstJointPower = firstJointPID.feedback(firstJointError);
        }
        else if (gamepad.left_stick_y == 0){
            firstJointPower = firstJointPID.feedback(firstJointError);
        }
        else {
            firstJointPower = -gamepad.left_stick_y;
            if (Math.abs(firstJointError) > MAX_CORRECT_AMOUNT) {
                // Reset
                firstJointTarget = robotHardware.firstJoint.getCurrentPosition();
            }

        }

        if (gamepad.right_stick_y == 0 && previousGamepad.right_stick_y!= 0) {
            secondJointTarget = robotHardware.secondJoint.getCurrentPosition();
            secondJointPID.reset();
            secondJointPower = 0;
        }
        else if (gamepad.right_stick_y == 0){
            secondJointPower = secondJointPID.feedback(secondJointError);
        }
        else {
            secondJointPower = gamepad.right_stick_y;
            if (Math.abs(secondJointError) > MAX_CORRECT_AMOUNT) {
                secondJointTarget = robotHardware.secondJoint.getCurrentPosition();
            }
        }


        robotHardware.firstJoint.setPower(MANUAL_SPEED * firstJointPower);
        robotHardware.secondJoint.setPower(MANUAL_SPEED * secondJointPower);


        telemetry.addData("first joint target", firstJointTarget);
        telemetry.addData("second joint target", secondJointTarget);
        telemetry.addData("first joint error", firstJointError);
        telemetry.addData("second joint error", secondJointError);
        try {
            previousGamepad.copy(gamepad);
        }
        catch (RobotCoreException e) {
            RobotLog.e(e.toString());
        }
    }

    void autoScalePower(double firstJointPower, double secondJointPower, double maxSpeed) {

        double scaledFirstJointPower, scaledSecondJointPower;

        if (Math.abs(firstJointPower) > Math.abs(secondJointPower)) {
            scaledFirstJointPower = maxSpeed * Math.signum(firstJointPower);
            scaledSecondJointPower = maxSpeed * secondJointPower / Math.abs(firstJointPower);
        }
        else {
            scaledSecondJointPower = maxSpeed * Math.signum(secondJointPower);
            scaledFirstJointPower = maxSpeed * firstJointPower / Math.abs(secondJointPower);
        }


        telemetry.addData("first joint power", scaledFirstJointPower);
        telemetry.addData("second joint power", scaledSecondJointPower);

        robotHardware.firstJoint.setPower(scaledFirstJointPower);
        robotHardware.secondJoint.setPower(scaledSecondJointPower);

    }

    public void setPositions(int firstJointTarget, int secondJointTarget) {
        this.firstJointTarget = firstJointTarget;
        this.secondJointTarget = secondJointTarget;
    }

    public void setPositions(ArmSetpoints setpoint) {
        setPositions(setpoint.firstJoint, setpoint.secondJoint);
    }


    /**
     *
     * @param setpoint
     * @param delay how long in milliseconds after the first joint starts moving that the second joint will follow (or vise-versa)
     */
    public void setPositionsWithDelay(ArmSetpoints setpoint, int delay) {
        if (delay == 0) {
            setPositions(setpoint);
        }
        if (delay > 0) {
            setPositions(setpoint.firstJoint, (int)secondJointTarget);
            Thread moveSecondJoint = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(delay);
                    }
                    catch (InterruptedException e) {
                        return;
                    }
                    setPositions(setpoint);
                }
            });
            moveSecondJoint.start();
        }
        else {
            setPositions((int)firstJointTarget, setpoint.secondJoint);
            Thread moveFirstJoint = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(Math.abs(delay));
                    }
                    catch (InterruptedException e) {
                        return;
                    }
                    setPositions(setpoint);
                }
            });
            moveFirstJoint.start();
        }
    }

    public boolean reachedTarget() {
        boolean firstJoint = Math.abs(firstJointTarget - robotHardware.firstJoint.getCurrentPosition()) < TARGET_TOLERANCE;
        boolean secondJoint = Math.abs(secondJointTarget - robotHardware.firstJoint.getCurrentPosition()) < TARGET_TOLERANCE;
        return firstJoint && secondJoint;
    }

    public void updateArmAuto() {
        updateArmAuto(-RobotConstants.MOVE_TO_SETPOINT_SPEED, RobotConstants.MOVE_TO_SETPOINT_SPEED);
    }
    public void updateArmAuto(double min, double max) {
        updateArmAuto(min, max, false);
    }

    private double firstJointStartingPosition;
    private double secondJointStartingPosition;
    private double wristStartingPosition;

    private void recordStartingPosition() {
        firstJointStartingPosition = robotHardware.firstJoint.getCurrentPosition();
        secondJointStartingPosition = robotHardware.secondJoint.getCurrentPosition();
        wristStartingPosition = robotHardware.intakeJoint.getCurrentPosition();
    }

    double WRIST_ENCODER_RATIO = 800;
    double kR = 0.001;

    CSVWriter writer = null;
    public void logCSV(String[] values) {
        if (writer == null) {
            try {
                writer = new CSVWriter(new FileWriter("/FIRST/log.csv"));
            } catch (IOException e) {
                RobotLog.e("Couldn't open csv log file");
                return;
            }
        }
        writer.writeNext(values);
    }

    public void closeCSV() {
        if (writer != null) {
            try {
                writer.close();
            }
            catch (IOException e) {
                RobotLog.e("Can't close csv log file");
            }
        }
    }

    public void updateArmAuto(double min, double max, boolean holdWristLevel) {
        double firstJointPosition = robotHardware.firstJoint.getCurrentPosition();
        double secondJointPosition = robotHardware.secondJoint.getCurrentPosition();
        double firstJointError = firstJointTarget - firstJointPosition;
        double secondJointError = secondJointTarget - secondJointPosition;

        double firstJointFeedback = firstJointPID.feedback(firstJointError);
        double secondJointFeedback = secondJointPID.feedback(secondJointError);
        telemetry.addData("First joint speed", firstJointFeedback);
        telemetry.addData("Second joint speed", secondJointFeedback);
        telemetry.addData("First joint position", firstJointPosition);
        telemetry.addData("Second joint position", secondJointPosition);
        telemetry.addData("First joint error", firstJointError);
        telemetry.addData("Second joint error", secondJointError);
        RobotLog.i(String.valueOf(firstJointFeedback));
        RobotLog.i(String.valueOf(secondJointFeedback));
        robotHardware.firstJoint.setPower(Range.clip(firstJointFeedback, min, max));
        robotHardware.secondJoint.setPower(Range.clip(secondJointFeedback, min, max));
    }

    void sweep(double firstJointAngularVelocity) {
        double secondJointAngularVelocity = 2 * firstJointAngularVelocity;
        double wristJointAngularVelocity = firstJointAngularVelocity + secondJointAngularVelocity;
        double firstJointSpeed = firstJointAngularVelocity *  FIRST_JOINT_ENCODER_RATIO;
        double secondJointSpeed = secondJointAngularVelocity * SECOND_JOINT_ENCODER_RATIO;
        double wristJointSpeed = wristJointAngularVelocity * WRIST_ENCODER_RATIO;

    }
}
