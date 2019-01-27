package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.arm.armkinematics.PIDController;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

public class JointControllerMotor extends JointController{

    DcMotorEx joint;
    public static double kp = 0;
    PIDController controller = new PIDController(0.0125, 0, 0);

    double maxSpeed = 0.75;

    public JointControllerMotor(RobotHardware rw) {
        super(rw);
        if (kp != 0) {
            controller = new PIDController(kp, 0,0);
        }
        joint = rw.intakeJoint;
    }

    // Set target position in degrees
    public void setAbsoluteTargetPosition(double targetPosition) {
        position = targetPosition;
    }

    public double update() {
        double feedback = controller.feedback(position - getAngle());
        double power = Range.clip(feedback,-maxSpeed, maxSpeed);
        joint.setPower(-power);
        return power;
    }

    public double holdInPlace(int position) {
        double power = Range.clip(controller.feedback(joint.getCurrentPosition() - position), -maxSpeed, maxSpeed);
        joint.setPower(-power);
        return power;
    }
}
