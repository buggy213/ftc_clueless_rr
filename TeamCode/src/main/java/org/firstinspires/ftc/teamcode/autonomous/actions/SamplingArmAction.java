package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;

import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.SAMPLING_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.drivetrain_test.RobotConstants.SAMPLING_SERVO_UP;

public class SamplingArmAction extends TimeBasedAction {



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
