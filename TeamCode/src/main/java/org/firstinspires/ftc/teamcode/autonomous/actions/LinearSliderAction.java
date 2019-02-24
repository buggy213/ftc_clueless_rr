package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

public class LinearSliderAction extends TimeBasedAction {
    RobotHardware rw;
    public LinearSliderAction(double startTime, double endTime, RobotHardware rw) {
        super(startTime, endTime);
        this.rw = rw;
    }

    @Override
    public void start() {
        rw.linearSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rw.linearSlider.setPower(1);
        super.start();
    }

    @Override
    public void stop() {
        rw.linearSlider.setPower(0);
        rw.linearSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
