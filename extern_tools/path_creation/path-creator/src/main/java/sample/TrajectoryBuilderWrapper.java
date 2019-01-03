package sample;

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
        this.name = name;
        poses = pose2dList;
        this.constraints = constraints;
    }
    public TrajectoryBuilderWrapper() {

    }

    public TrajectoryBuilder toTrajectoryBuilder() {
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(poses.get(0), constraints, 2500);
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
            }
            i++;
        }
        return trajectoryBuilder;
    }
}
