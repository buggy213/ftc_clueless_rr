package org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize;

import com.acmerobotics.roadrunner.Pose2d;

public class Pose2dWrapper {

    public double heading;
    public double x;
    public double y;

    public Pose2dWrapper(Pose2d pose) {
        heading = pose.getHeading();
        x = pose.getX();
        y = pose.getY();
    }

    public Pose2dWrapper() {

    }
}
