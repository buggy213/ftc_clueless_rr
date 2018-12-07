package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;

public class SamplingArmAction extends TimeBasedAction {

    final double SAMPLING_SERVO_UP = 1;
    final double SAMPLING_SERVO_DOWN = 0.55;

    RobotHardware rw;

    public SamplingArmAction(double startTime, double endTime, RobotHardware robotHardware){
        super(startTime, endTime);
        this.rw = robotHardware;
    }

    @Override
    public void start() {
        rw.samplingServo.setPosition(SAMPLING_SERVO_DOWN);
        super.start();
    }

    @Override
    public void stop() {
        rw.samplingServo.setPosition(SAMPLING_SERVO_UP);
        super.stop();
    }
}
