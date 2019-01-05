package org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryBuilderWrapper {

    public DriveConstraintsWrapper driveConstraintsWrapper;
    public List<Pose2dWrapper> pose2dWrapper;
    public List<Options> options;
    public String name;

    private List<Pose2d> poses;
    private DriveConstraints constraints;

    public TrajectoryBuilderWrapper(List<Pose2d> pose2dList, List<Options> options, DriveConstraints constraints, String name) {
        driveConstraintsWrapper = new DriveConstraintsWrapper(constraints);
        pose2dWrapper = new ArrayList<>();
        for (Pose2d pose : pose2dList) {
            pose2dWrapper.add(new Pose2dWrapper(pose));
        }

        this.options = options;
        poses = pose2dList;
        this.constraints = constraints;
    }
    public TrajectoryBuilderWrapper() {

    }

    public TrajectoryBuilder toTrajectoryBuilder() {
        return toTrajectoryBuilder(TrajectoryTransform.identity());
    }

    public TrajectoryBuilder toTrajectoryBuilder(TrajectoryTransform transform) {
        Pose2dWrapper first = pose2dWrapper.get(0);
        return toTrajectoryBuilder(new Pose2d(first.x, first.y, first.heading), transform);
    }

    public TrajectoryBuilder toTrajectoryBuilder(Pose2d poseEstimate, TrajectoryTransform transform) {
        poses = new ArrayList<>();
        for (Pose2dWrapper pose : pose2dWrapper) {
            poses.add(transform.transform(pose));
        }
        constraints = new DriveConstraints(driveConstraintsWrapper.maxVel, driveConstraintsWrapper.maxAcc, driveConstraintsWrapper.maxAngleVel, driveConstraintsWrapper.maxAngleAcc);

        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(poseEstimate, constraints, 2500);
        int i = 0;
        for (Options o : options) {
            switch(o) {
                case lineTo:
                    trajectoryBuilder.lineTo(poses.get(i + 1).pos());
                    break;
                case splineTo:
                    trajectoryBuilder.splineTo(poses.get(i + 1));
                    break;
                case turnTo:
                    trajectoryBuilder.turnTo(poses.get(i + 1).getHeading());
                    break;
                case reverse:
                    trajectoryBuilder.reverse();
                    break;
                case strafeTo:
                    trajectoryBuilder.strafeTo(poses.get(i + 1).pos());
                    break;
            }
            i++;
        }
        return trajectoryBuilder;
    }
}
