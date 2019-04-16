package org.firstinspires.ftc.teamcode.shared;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

@Config
public class RobotConstants {
    public static double MOVE_TO_SETPOINT_SPEED = 1;
    public static double INTAKE_WRIST_SPEED = 0.5;
    public static double SWEEP_SPEED = 0.7;
    public static double SWEEP_SPEED_TELEOP = 0.8;
    public static double CONTOUR_MIN_AREA = 500;

    // Servo stuff (don't touch)
    public static double LOCK_ENGAGED = 0.37;
    public static double LOCK_DISENGAGED = 0.65;

    public static double INTAKE_JOINT_UP = 0.275;
    public static double INTAKE_JOINT_DOWN = 0.15;

    public static double BACK_SERVO_UP = 0.45;
    public static double BACK_SERVO_DOWN = 0;

    public static double SAMPLING_SERVO_DOWN = 0.45;
    public static double SAMPLING_SERVO_UP = 0.889;

    public static double DOOR_OPEN = 0.33;
    public static double DOOR_CLOSED = 0.885;
    public static double DOOR_CUBE_RELEASE = 0.59;

    // Grabbing 2 at once
    public static double WIDE_CLAW_LEFT = 0.69;
    public static double WIDE_CLAW_RIGHT = 0.29;

    // Grabbing 1
    public static double NARROW_CUBE_CLAW_LEFT = 0.475;
    public static double NARROW_CUBE_CLAW_RIGHT = 0.595;

    // Speed of intake motor at end of arm
    public static double INTAKE_SPEED = 0.5;

    // Speed of arm when in manual control (teleop)
    public static double MANUAL_SPEED = 10;

    public static PIDCoefficients FIRST_JOINT_PID = new PIDCoefficients(0.01, 0,0);
    public static PIDCoefficients SECOND_JOINT_PID = new PIDCoefficients(0.01, 0,0 );
    public static double INTAKE_HOLD_ANGLE = 0;
}
