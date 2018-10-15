package org.clueless.motionplanning.planning.kinematics;

// Based on http://web.eecs.umich.edu/~ocj/courses/autorob/autorob_10_ik_closedform.pdf
public class TwoJointedArmKinematics {
    double length1;
    double length2;
    public double[] inverseKinematics(double xe, double ye, double angle) {
        double theta2 = Math.PI - Math.acos((Math.sqrt((Math.pow(xe, 2)) + Math.pow(ye, 2)) - Math.pow(length1, 2) - Math.pow(length2, 2))/
                (2 * length1 * length2));
        double theta1 = Math.atan(ye / xe) - Math.atan((length2 * Math.sin(theta2)) / (length1 + length2 * Math.cos(theta2)));

        return new double[] {
                theta1,
                theta2
        };
    }
}
