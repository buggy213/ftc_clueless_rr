package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_DOWN;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_UP;

public class IntakeAction extends TimeBasedAction {
    RobotHardware rw;

    public IntakeAction(double startTime, double endTime, RobotHardware robotHardware){
        super(startTime, endTime);
        this.rw = robotHardware;
    }

    @Override
    public void start() {
        rw.intakeJoint.setPosition(INTAKE_JOINT_DOWN);
        super.start();
    }

    @Override
    public void stop() {
        rw.intakeJoint.setPosition(INTAKE_JOINT_UP);
        super.stop();
    }
}
