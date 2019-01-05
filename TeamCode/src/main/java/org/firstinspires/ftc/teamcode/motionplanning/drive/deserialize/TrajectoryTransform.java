package org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

public class TrajectoryTransform {
    private double rotation;
    private Vector2d translation;

    public TrajectoryTransform() {
        this(0, new Vector2d(0, 0));
    }

    public TrajectoryTransform(double rotation, Vector2d translation) {
        this.rotation = rotation;
        this.translation = translation;
    }

    public static TrajectoryTransform identity() {
        return new TrajectoryTransform(0, new Vector2d(0,0));
    }

    public static TrajectoryTransform oneEighty() {
        return new TrajectoryTransform(Math.PI, new Vector2d(0 ,0));
    }

    public Pose2d transform(Pose2dWrapper wrapper) {
        return transform(new Pose2d(wrapper.x, wrapper.y, wrapper.heading));
    }

    public Pose2d transform(Pose2d pose) {
        // Rotate, and then translate
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double x_prime = x * Math.cos(rotation) - y * Math.sin(rotation);
        double y_prime = y * Math.cos(rotation) + x * Math.sin(rotation);
        x_prime += translation.getX();
        y_prime += translation.getY();

        heading += rotation;

        return new Pose2d(x_prime, y_prime, heading);
    }
}
