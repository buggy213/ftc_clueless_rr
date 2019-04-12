package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.SAMPLING_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.SAMPLING_SERVO_UP;

public class LiftArmAction extends TimeBasedAction {



    RobotHardware rw;

    public LiftArmAction(double startTime, double endTime, RobotHardware robotHardware){
        super(startTime, endTime);
        this.rw = robotHardware;
    }

    @Override
    public void start() {
        rw.firstJoint.setPower(-1);
        super.start();
    }

    @Override
    public void stop() {
        rw.firstJoint.setPower(0);
        super.stop();
    }
}
