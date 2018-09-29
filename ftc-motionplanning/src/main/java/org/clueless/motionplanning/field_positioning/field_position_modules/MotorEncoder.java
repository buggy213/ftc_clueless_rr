package org.clueless.motionplanning.field_positioning.field_position_modules;


import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorEncoder extends Encoder {

    DcMotor motor;

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public MotorEncoder(DcMotor motor) {
        this.motor = motor;
    }
}
