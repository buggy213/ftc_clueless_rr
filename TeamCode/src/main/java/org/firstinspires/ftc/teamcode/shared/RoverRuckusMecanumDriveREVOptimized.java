package org.firstinspires.ftc.teamcode.shared;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.motionplanning.drive.config.DriveConstants;
import org.firstinspires.ftc.teamcode.motionplanning.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.motionplanning.util.DashboardUtil;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.autonomous.Autonomous.debug;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings here are enough to cut loop
 * iteration times in half which may significantly improve trajectory following performance.
 */
public class RoverRuckusMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private static BNO055IMU imu;


    private final boolean reset = false;
    private double offset;

    PIDCoefficients pidCoefficients = new PIDCoefficients(0.1, 0, 50);

    public RoverRuckusMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        this(hardwareMap, 0);
    }

    public RoverRuckusMecanumDriveREVOptimized(HardwareMap hardwareMap, double offset) {
        super();

        this.offset = offset;

        // TODO: adjust the names of the following hardware devices to match your configuration
        if (imu == null || reset) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        setLocalizer(new MecanumLocalizer(this, true));

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }



    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        /*
            TelemetryPacket packet = new TelemetryPacket();
            packet.addLine("Gyro: " + getExternalHeading());
            packet.addLine("v: " + v);
            packet.addLine("v1: " + v1);
            packet.addLine("v2: " + v2);
            packet.addLine("v3: " + v3);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        */
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getExternalHeading() {
        return positiveModulo(imu.getAngularOrientation().firstAngle + offset, 2 * Math.PI);
    }

    public double positiveModulo(double a, double b) {
        return ((a % b) + b) % b;
    }

    public void displayPosition(Trajectory trajectory) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Pose2d currentPose = getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

        fieldOverlay.setStrokeWidth(4);
        fieldOverlay.setStroke("green");
        DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            dashboard.sendTelemetryPacket(packet);
    }

    public boolean verifyTrajectoryConfig(TrajectoryConfig config) {
        return (config.getConstraints() == DriveConstants.BASE_CONSTRAINTS);
    }
}
