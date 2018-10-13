package org.clueless.motionplanning.math;

public class TwoDimensionalTransform {
    public Vector2 vector;
    public double angle;

    public void translate(Vector2 translation) {
        vector = Vector2.add(vector, translation);
    }

    public void rotate(double angle) {
        this.angle += angle;
    }

    public TwoDimensionalTransform(Vector2 position, double angle) {
        this.vector = position;
        this.angle = angle;
    }
}
