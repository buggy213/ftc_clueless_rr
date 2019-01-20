package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_DOWN;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_UP;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.NARROW_CUBE_CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.NARROW_CUBE_CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.WIDE_CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.WIDE_CLAW_RIGHT;

public class IntakeAction extends TimeBasedAction {
    RobotHardware rw;

    public IntakeAction(double startTime, double endTime, RobotHardware robotHardware){
        super(startTime, endTime);
        this.rw = robotHardware;
    }

    @Override
    public void start() {
        // TODO fix marker
        // rw.intakeJoint.setPosition(INTAKE_JOINT_DOWN);
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
