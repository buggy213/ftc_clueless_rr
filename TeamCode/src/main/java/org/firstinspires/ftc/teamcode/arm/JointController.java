package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shared.RobotHardware;

import java.util.LinkedList;

public abstract class JointController {
    BNO055IMU imu;

    private double offset;
    protected  boolean useImu;
    protected static final String IMU_NAME = "jointImu";
    protected static final String JOINT_NAME = "intakeJoint";
    double position;

    public JointController(RobotHardware rw, boolean useImu) {
        if (useImu) {
            initializeIMU(rw.hardwareMap);
            this.useImu = useImu;
        }
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, IMU_NAME);
        imu.initialize(parameters);
    }

    public void setOffset(double offset) { this.offset = offset; }
    public double getAngle() {
        return imu.getAngularOrientation().firstAngle + offset;
    }
}
