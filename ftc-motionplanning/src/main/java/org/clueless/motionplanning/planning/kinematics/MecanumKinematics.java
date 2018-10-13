package org.clueless.motionplanning.planning.kinematics;

// Based on https://www.chiefdelphi.com/media/papers/2390 paper by Ether (?)

import org.clueless.motionplanning.math.TwoDimensionalTransform;
import org.clueless.motionplanning.math.Vector2;

import Jama.Matrix;

public class MecanumKinematics {

    /**
     *
     * @param desiredRobotVelocity
     * @param length Distance between wheels on the same side of the robot
     * @param width Distance between wheels on opposite sides of the robot
     * @return Array of unscaled wheel velocities, going from forward left and proceeding anticlockwise around the robot
     */
    double[] robotVelocityToWheelVelocities(TwoDimensionalTransform desiredRobotVelocity, double width, double length) {
        // Assume rollers are in following configuration https://slideplayer.com/slide/8874970/26/images/2/Mecanum+Wheels.jpg
        return new double[] {
                desiredRobotVelocity.vector.y - width * desiredRobotVelocity.angle / 2 - (desiredRobotVelocity.vector.x + length * desiredRobotVelocity.angle / 2),
                desiredRobotVelocity.vector.y - width * desiredRobotVelocity.angle / 2 + (desiredRobotVelocity.vector.x - length * desiredRobotVelocity.angle / 2),
                desiredRobotVelocity.vector.y + width * desiredRobotVelocity.angle / 2 - (desiredRobotVelocity.vector.x - length * desiredRobotVelocity.angle / 2),
                desiredRobotVelocity.vector.y + width * desiredRobotVelocity.angle / 2 + (desiredRobotVelocity.vector.x + length * desiredRobotVelocity.angle / 2)
        };
    }

    TwoDimensionalTransform wheelVelocitiesToRobotVelocity(double[] wheelVelocities, double width, double length) {
        double k = (Math.abs(width) + Math.abs(length)) / 2;
        double l = 1 / (4 * k);
        double[][] forwardMatrixValues = {
                {0.25, 0.25, 0.25, 0.25},
                {-0.25, 0.25, -0.25, 0.25},
                {-l, -l, l, l}
        };

        Matrix forwardMatrix = new Matrix (forwardMatrixValues);
        Matrix wheelVelocitiesMatrix = new Matrix(wheelVelocities, 1);
        Matrix product = forwardMatrix.times(wheelVelocitiesMatrix);

        return new TwoDimensionalTransform(new Vector2(product.get(0 ,0), product.get(1, 0)), product.get(2, 0));
    }

}
