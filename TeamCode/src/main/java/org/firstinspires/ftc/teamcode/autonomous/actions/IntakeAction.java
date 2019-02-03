package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.JointControllerMotor;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_DOWN;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_UP;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_SPEED;
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
        rw.intakeJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.intakeJoint.setTargetPosition(850);
        rw.intakeJoint.setPower(-0.25);
    }

    @Override
    public void update() {
        while(timer.time() < end) {
            rw.intake.setPower(INTAKE_SPEED * 1.25);
        }
        rw.intakeJoint.setTargetPosition(0);
        rw.intakeJoint.setPower(0.25);
        stop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
