package org.firstinspires.ftc.teamcode.motionplanning.drive.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

/*
 * Constants shared between multiple drive types.
 */
@Config
public class DriveConstants {

    private DriveConstants() {

    }

    /*
     * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
     * fields may also be edited through the dashboard (connect to the robot's WiFi network and
     * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
     * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
     */
    public static final Neverest40GearmotorConfig MOTOR_CONFIG = new Neverest40GearmotorConfig();
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double WHEEL_RADIUS = 2.3; // in
    public static double GEAR_RATIO = 0.66666; // output/input

    // TODO figure out why track width calibration outputs 0.51
    public static double TRACK_WIDTH = 14; // in

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(20, 16, Math.PI / 2, Math.PI / 4);

    public static double kV = 0.0236;
    public static double kA = 0;
    public static double kStatic = 0;


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * DriveConstants.GEAR_RATIO * 2 * Math.PI * DriveConstants.WHEEL_RADIUS / 60.0;
    }
}
