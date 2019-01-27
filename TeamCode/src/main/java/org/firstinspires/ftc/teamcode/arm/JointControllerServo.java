package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import java.util.LinkedList;

public class JointControllerServo extends JointController{


    private Servo joint;

    private static final double ANGLE_THRESHOLD = 2.5;
    private static final int ANGLE_HISTORY = 10;

    double currentAnglePlurality;
    LinkedList<Double> previousAngles;

    public JointControllerServo(RobotHardware rw) {
        super(rw);

        joint = rw.hardwareMap.get(Servo.class, JOINT_NAME);
        previousAngles = new LinkedList<>();
    }

    public void setTargetRelativePosition(double degrees) {
        if (degrees > 90 || degrees < -90) {
            RobotLog.e("Joint angle out of range");
        }

        joint.setPosition(0.5 * degrees / 90 + 0.5);
    }

    public void setTargetAbsolutePosition(double degrees) {
        if (degrees > 90 || degrees < -90) {
            RobotLog.e("Joint angle out of range");
        }

        position = degrees;
    }

    public void update() {
        if (previousAngles.size() >= ANGLE_HISTORY) {
            previousAngles.removeFirst();
        }
        previousAngles.add(getAngle());
        double currentAverage = average();
        if (Math.abs(currentAverage - currentAnglePlurality) > ANGLE_THRESHOLD) {
            currentAnglePlurality = currentAverage;
            setTargetRelativePosition(currentAnglePlurality);
        }

    }

    private double average() {
        double sum = 0;
        for (Double d : previousAngles) {
            sum += d;
        }

        sum /= previousAngles.size();
        return sum;
    }
}
