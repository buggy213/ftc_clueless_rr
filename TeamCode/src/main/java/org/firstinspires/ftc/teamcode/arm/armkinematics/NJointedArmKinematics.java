package org.firstinspires.ftc.teamcode.arm.armkinematics;


import com.acmerobotics.roadrunner.Vector2d;

// Cyclic coordinate descent based IK solver
public class NJointedArmKinematics {
    int n;
    Vector2d e;
    Vector2d[] jointPositions;
    double[] jointAngles;
    double[] lengths;
    public NJointedArmKinematics(int numJoints, double[] lengths) {
        n = numJoints;
        jointPositions = new Vector2d[n];
        double sum = 0;
        jointAngles = new double[n];
        jointPositions[0] = new Vector2d(0,0);
        this.lengths = lengths;
        for (int i = 1; i < n; i++) {
            sum += lengths[i - 1];
            jointPositions[i] = new Vector2d(sum, 0);
        }
        sum += lengths[n];
        e = new Vector2d(sum, 0);
    }
// http://www.ryanjuckett.com/programming/cyclic-coordinate-descent-in-2d/
    void step(Vector2d target) {

        for(int i = 0; i < n; i++) {
            double cos_a = ((e.minus(jointPositions[i])).dot((target.minus(jointPositions[i])))) /
                            (e.minus(jointPositions[i]).norm() * target.minus(jointPositions[i]).norm());

            double sin_a = (((e.getX() - jointPositions[i].getX()) * (target.getY() - jointPositions[i].getY()) - ((e.getY() - jointPositions[i].getY()) * (target.getX() - jointPositions[i].getX())))/
                            ((e.minus(jointPositions[i]).norm() * target.minus(jointPositions[i]).norm())));

            double a = Math.acos(cos_a) * ((sin_a > 0) ? 1 : -1);

            jointAngles[i] = a;

            recalculateJointPositions();
        }
    }

    void recalculateJointPositions() {

        Vector2d currentJointPosition = new Vector2d(0,0 );
        for(int i = 1; i < jointAngles.length; i++) {
            currentJointPosition.plus(new Vector2d(Math.cos(jointAngles[i]), Math.sin(jointAngles[i])).times(lengths[i - 1]));
        }

        e = currentJointPosition.plus(new Vector2d(Math.cos(jointAngles[n]), Math.sin(jointAngles[n])).times(lengths[n]));
    }

    public double inverseKinematics(double epsilon, int maxIterations) {
        return 0;
    }
}
