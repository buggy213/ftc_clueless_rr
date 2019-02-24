package org.firstinspires.ftc.teamcode.autonomous.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.ArmSetpoints;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.SWEEP_SPEED_TELEOP;

@TeleOp(name="auto score test")
public class AutoScoreConcept extends LinearOpMode {
    double firstJointKR;
    double secondJointKR;

    ArmController armController;
    RobotHardware rw;

    @Override
    public void runOpMode() {
        rw = new RobotHardware(hardwareMap);
        armController = new ArmController(rw, telemetry, true);
        while (!isStarted()) {

        }
        while (opModeIsActive()) {
            if (gamepad1.a) {
                score();
            }
            if (gamepad1.b) {
                moveToCollect();
            }
            if (gamepad1.x) {
                sweep();
            }
        }
    }

    private void score() {
        armController.setPositions(ArmSetpoints.START);
        while (!armController.reachedTarget()) {
            armController.updateArmAuto();
        }
        armController.setPositions(ArmSetpoints.SCORE);
        while (!armController.reachedTarget()) {
            armController.updateArmAuto();
        }
    }

    private void moveToCollect() {
        armController.setPositions(ArmSetpoints.COLLECT);
        while(!armController.reachedTarget()) {
            armController.updateArmAuto();
        }
    }

    private void sweep() {
        rw.intake.setPower(-SWEEP_SPEED_TELEOP);
        rw.firstJoint.setPower(-0.2);
        rw.secondJoint.setPower(0.3);

    }
}
