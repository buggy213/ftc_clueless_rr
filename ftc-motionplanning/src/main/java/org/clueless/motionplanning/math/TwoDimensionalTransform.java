package org.clueless.motionplanning.math;

public class TwoDimensionalTransform {
    public Vector2 position;
    public double angle;

    public void translate(Vector2 translation) {
        position = Vector2.add(position, translation);
    }

    public void rotate(double angle) {
        this.angle += angle;
    }


}
