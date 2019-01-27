package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.arm.JointControllerMotor;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_DOWN;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_UP;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.NARROW_CUBE_CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.NARROW_CUBE_CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.WIDE_CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.WIDE_CLAW_RIGHT;

public class IntakeAction extends TimeBasedAction {
    RobotHardware rw;
    JointControllerMotor joint;
    public IntakeAction(double startTime, double endTime, RobotHardware robotHardware){
        super(startTime, endTime);
        this.rw = robotHardware;
        joint = new JointControllerMotor(rw);
    }

    @Override
    public void start() {
        // TODO fix marker
        // rw.intakeJoint.setPosition(INTAKE_JOINT_DOWN);
        super.start();
        joint.setAbsoluteTargetPosition(-161);

    }

    @Override
    public void update() {
        while (timer.time() < (end - start) / 2) {
            joint.update();
        }
        joint.setAbsoluteTargetPosition(0);
        while(timer.time() < end) {
            joint.update();
        }
        stop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
