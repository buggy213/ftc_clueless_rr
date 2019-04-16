package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shared.RobotConstants;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

@TeleOp(name="Arm Test")
public class ArmTest extends OpMode {

    RobotHardware rw;
    ArmController armController;

    @Override
    public void loop() {
        telemetry.addData("First Joint Encoder", rw.firstJoint.getCurrentPosition());
        telemetry.addData("Second Joint Encoder", rw.secondJoint.getCurrentPosition());
        telemetry.addData("Wrist Joint Encoder", rw.intakeJoint.getCurrentPosition());

        double a = 147 + (rw.firstJoint.getCurrentPosition() / 1126.817) * (180 / Math.PI);
        double b = -170 -  (rw.secondJoint.getCurrentPosition() / 951.747) * (180 / Math.PI);
        double c = 180 - (rw.intakeJoint.getCurrentPosition() / 551.313) * (180 / Math.PI);
        telemetry.addData("First Joint Angle (estimated)", a);
        telemetry.addData("Second Joint Angle (estimated)", b);
        telemetry.addData("Wrist Joint Angle (estimated)", c);

        telemetry.addData("needed third joint angle", -(a+b) + 55);
        telemetry.addData("third joint encoder target", 551.313 * Math.PI / 180 * (a+b+125));
        double wristPower = gamepad1.left_bumper ? RobotConstants.INTAKE_WRIST_SPEED : (gamepad1.right_bumper ? -RobotConstants.INTAKE_WRIST_SPEED : 0);
        rw.intakeJoint.setPower(wristPower);
        armController.manualArmControl(gamepad1);
        telemetry.update();
    }

    @Override
    public void init() {
        rw = new RobotHardware(hardwareMap);
        armController = new ArmController(rw, telemetry, true);
    }
}
