package sample;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

public class DriveConstraintsWrapper {
    public double maxVel;
    public double maxAcc;
    public double maxAngleVel;
    public double maxAngleAcc;

    public DriveConstraintsWrapper(DriveConstraints d) {
        maxVel = d.maximumVelocity;
        maxAcc = d.maximumAcceleration;
        maxAngleVel = d.maximumAngularVelocity;
        maxAngleAcc = d.maximumAngularAcceleration;
    }

    public DriveConstraintsWrapper() {

    }
}
