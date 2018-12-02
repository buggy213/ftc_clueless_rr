package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.motionplanningtest.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.motionplanningtest.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.LynxOptimizedI2cFactory;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings here are enough to cut loop
 * iteration times in half which may significantly improve trajectory following performance.
 */
public class RoverRuckusMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    private double offset;

    PIDCoefficients pidCoefficients = new PIDCoefficients(0.1, 0, 50);

    public RoverRuckusMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        this(hardwareMap, 0);
    }

    public RoverRuckusMecanumDriveREVOptimized(HardwareMap hardwareMap, double offset) {
        super();

        this.offset = offset;

        RevExtensions2.init();

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // note: this strategy is still applicable even if the drive motors are split between hubs
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

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
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();
        List<Double> wheelPositions = new ArrayList<>();
        boolean bulkDataExists = bulkData != null;
        for (ExpansionHubMotor motor : motors) {
            if (bulkDataExists) {
                wheelPositions.add(DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
            }
            else {
                wheelPositions.add(0d);
            }
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Gyro: " + getExternalHeading());
        packet.addLine("v: " + v);
        packet.addLine("v1: " + v1);
        packet.addLine("v2: " + v2);
        packet.addLine("v3: " + v3);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
}
