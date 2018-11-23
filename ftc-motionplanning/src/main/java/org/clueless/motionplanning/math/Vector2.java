package org.clueless.motionplanning.math;

public class Vector2 {
    public double x;
    public double y;

    public Vector2() {
        this(0, 0);
    }

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2 multiply (double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }

    public Vector2 dotProduct(Vector2 other) {
        return new Vector2(x * other.x, y * other.y);
    }

    public static Vector2 add(Vector2 first, Vector2 second) {
        return new Vector2(first.x + second.x, first.y + second.y);
    }
}
