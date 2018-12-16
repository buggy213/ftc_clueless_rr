package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import java.util.LinkedList;

public class JointController {
    BNO055IMU imu;

    private static final String IMU_NAME = "jointImu";
    private static final String JOINT_NAME = "intakeJoint";

    private Servo joint;

    double position;

    private static final double ANGLE_THRESHOLD = 2.5;
    private static final int ANGLE_HISTORY = 10;

    double currentAnglePlurality;
    LinkedList<Double> previousAngles;

    public JointController(RobotHardware rw) {
        initializeIMU(rw.hardwareMap);
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



    private void initializeIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, IMU_NAME);
        imu.initialize(parameters);
    }

    private double getAngle() {
        return imu.getAngularOrientation().firstAngle;
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
