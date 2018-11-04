package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorServo{


    DcMotor motor;
    // Value from 0 to 2pi which represents the rotation of the motor
    double currentPosition;
    double kp, ki, kd;

    int targetPosition;

    static double encoderToPositionRatio;

    public void setPosition(double position) {

    }

    public void update() {
        double error =
    }


}
