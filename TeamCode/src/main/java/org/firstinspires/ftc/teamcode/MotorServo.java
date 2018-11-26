package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorServo{


    DcMotor motor;
    MotorConfiguration configuration;
    double kp, ki, kd;
    double previousError, errorSum;


    double target;
    public static class MotorConfiguration {

        public static MotorConfiguration firstJoint = new MotorConfiguration(1680, 1);
        public static MotorConfiguration secondJoint = new MotorConfiguration(1680, 1);

        public double encoderTicksPerRevolution;
        public MotorConfiguration(double baseRatio, double gearRatio) {
            encoderTicksPerRevolution = baseRatio * gearRatio;
        }
    }


    public void setPosition(double position, boolean clockwise) {
        target = position * configuration.encoderTicksPerRevolution;
        errorSum = 0;
        previousError = 0;
    }

    public void update() {
        double error = motor.getCurrentPosition() - target;
        double adjustment = kp * error + ki * errorSum + kd * (error - previousError);
        motor.setPower(adjustment);
        previousError = error;
        errorSum += error;

    }


}
