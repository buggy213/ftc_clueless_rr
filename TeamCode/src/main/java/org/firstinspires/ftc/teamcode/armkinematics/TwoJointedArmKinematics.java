package org.firstinspires.ftc.teamcode.armkinematics;

// Based on http://web.eecs.umich.edu/~ocj/courses/autorob/autorob_10_ik_closedform.pdf
public class TwoJointedArmKinematics {
    double length1;
    double length2;

    public TwoJointedArmKinematics(double length1, double length2) {
        this.length1 = length1;
        this.length2 = length2;
    }

    public double[] inverseKinematics(double xe, double ye) {
        if (xe == 0 && ye != 0) {
            return new double[] {
                    90,
                    0
            };
        }
        double a = (Math.pow(xe, 2)) + Math.pow(ye, 2) - Math.pow(length1, 2) - Math.pow(length2, 2);
        double theta2 = Math.acos(a / (2 * length1 * length2));
        double theta1 = signedAtan(xe, ye) - Math.atan((length2 * Math.sin(theta2)) / (length1 + length2 * Math.cos(theta2)));
        double secondtheta2 = -theta2;
        double secondtheta1 = theta1 + theta2;
        if (theta1 == 0) {
            double k = 1;
        }

        return new double[] {
                theta1,
                theta2,
                secondtheta1,
                secondtheta2
        };
    }

    double signedAtan(double x, double y) {
        if (x < 0) {
            return Math.atan(y / x) + Math.PI;
        }

        return Math.atan(y / x);
    }
}
