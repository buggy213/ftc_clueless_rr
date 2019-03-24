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
    int delay = 200;
    public IntakeAction(double startTime, double endTime, RobotHardware robotHardware){
        super(startTime, endTime);
        this.rw = robotHardware;
        joint = new JointControllerMotor(rw, false);
    }

    public void setDelay(int delay) {
        this.delay = delay;
    }

    @Override
    public void start() {
        // TODO fix marker
        // rw.intakeJoint.setPosition(INTAKE_JOINT_DOWN);
        rw.intakeJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.intakeJoint.setTargetPosition(700);
        rw.intakeJoint.setPower(-0.8);
        try {
            Thread.sleep(delay);
        }
        catch(InterruptedException e) {
            stop();
        }
        update();
    }

    @Override
    public void update() {
        while(timer.time() < end) {
            rw.intake.setPower(INTAKE_SPEED * 1.25);
        }
        stop();
    }

    @Override
    public void stop() {
        rw.intakeJoint.setTargetPosition(0);
        rw.intakeJoint.setPower(0.25);
        rw.intake.setPower(0);
        super.stop();
    }
}
