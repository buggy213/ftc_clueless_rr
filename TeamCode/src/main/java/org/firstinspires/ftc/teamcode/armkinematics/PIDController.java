package org.firstinspires.ftc.teamcode.armkinematics;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class PIDController {
    double kp, ki, kd;
    double sum;
    double previousError;
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public PIDController(PIDCoefficients coefficients) {
        this(coefficients.kP, coefficients.kI, coefficients.kD);
    }

    public double feedback(double error) {
        double value = kp * error + ki * sum + kd * (error - previousError);
        sum += error;
        previousError = error;
        return value;
    }
}
