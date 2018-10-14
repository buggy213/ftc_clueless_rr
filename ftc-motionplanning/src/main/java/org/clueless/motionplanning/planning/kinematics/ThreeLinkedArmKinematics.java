package org.clueless.motionplanning.planning.kinematics;
// Based on https://ocw.mit.edu/courses/mechanical-engineering/2-12-introduction-to-robotics-fall-2005/lecture-notes/chapter4.pdf
public class ThreeLinkedArmKinematics {
    double L1;
    double L2;
    double L3;
    // Inverse kinematic equations -- given desired position of end effector, find (2?) possible sets of joint angles
    // Implement equations 4.2.4, 4.2.6, and 4.2.7

    /**
     *
     * @param xe x coordinate of end effector on O-xy plane
     * @param ye y coordinate of end effector on O-xy plane
     * @param angle angle of end effector relative to O-xy plane
     */
    double[] inverseKinematics(double xe, double ye, double angle) {
        double xw = xe - L3 * Math.cos(angle);
        double yw = ye - L3 * Math.sin(angle);
        double theta2 = Math.PI - Math.acos((Math.pow(L1, 2) + Math.pow(L2, 2) - Math.pow(xw, 2) - Math.pow(yw, 2))
                / (2 * L1 * L2));
        double theta1 = Math.atan(yw / xw) - Math.acos((Math.pow(xw, 2) + Math.pow(yw, 2) + Math.pow(L1, 2) - Math.pow(L2, 2)
                / ((2 * L1 * Math.sqrt(Math.pow(xw, 2) + Math.pow(yw, 2))))));
        double theta3 = angle - theta1 - theta2;
        return new double[] {
                theta1,
                theta2,
                theta3
        };
    }
}
