package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.armkinematics.PIDController;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

public class JointControllerMotor extends JointController{

    DcMotorEx joint;
    public static double kp = 0;
    PIDController controller = new PIDController(0.0175, 0, 0);

    double previousError;
    double maxSpeed = 0.75;

    public JointControllerMotor(RobotHardware rw, boolean useImu) {
        super(rw, useImu);
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
        if (!useImu) {
            return 0;
        }

        double angle = getAngle();

        // Preventing rollover / spazzyboi
        if (angle > 90) {
            angle = -360 + angle;
        }


        double error = position - angle;

        double feedback = controller.feedback(error);
        double power = Range.clip(feedback,-maxSpeed, maxSpeed);
        joint.setPower(-power);

        return power;
    }

    public double holdInPlace(int position) {
        double power = Range.clip(controller.feedback(joint.getCurrentPosition() - position), -maxSpeed, maxSpeed);
        joint.setPower(-0.75*power);
        return power;
    }

    // encoder counts / radian
    double firstJointRatio = 1126.817;
    double secondJointRatio = 951.747;
    double thirdJointRatio = 551.313;
    public void holdLevel(RobotHardware rw, Telemetry t) {
        double a = 147 + (rw.firstJoint.getCurrentPosition() / firstJointRatio) * (180 / Math.PI);
        double b = -170 -  (rw.secondJoint.getCurrentPosition() / secondJointRatio) * (180 / Math.PI);
        double c = 180 - (rw.intakeJoint.getCurrentPosition() / thirdJointRatio) * (180 / Math.PI);
        double d = 551.313 * Math.PI / 180 * (a+b+125);
        double power = (d - rw.intakeJoint.getCurrentPosition()) * 0.002;
        t.addData("desired wrist joint encoder count", d);
        joint.setPower(power);
    }
}
