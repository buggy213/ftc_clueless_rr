package org.clueless.motionplanning.math;

public class Units {
    public static double toMm(double inches) {
        return inches * 25.4;
    }

    public static double toInches(double mm) {
        return mm / 25.4;
    }

    public static double ftToMm(double feet) {
        return toMm(feet * 12);
    }
}
