package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.JointControllerMotor;
import org.firstinspires.ftc.teamcode.arm.JointControllerServo;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

@TeleOp(name = "Joint controller test")
public class JointControllerTest extends LinearOpMode {
    JointControllerMotor jointController;

    @Override
    public void runOpMode() {
        waitForStart();
        RobotHardware rw = new RobotHardware(hardwareMap);
        jointController = new JointControllerMotor(rw, true);

        jointController.setAbsoluteTargetPosition(0);


        while (opModeIsActive()) {
            double feedback = jointController.update();
            telemetry.addData("Angle", jointController.getAngle());
            telemetry.addData("feedback", feedback);
            telemetry.update();
        }



    }
}
