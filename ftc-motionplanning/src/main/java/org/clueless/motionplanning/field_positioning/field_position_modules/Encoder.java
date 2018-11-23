package org.clueless.motionplanning.field_positioning.field_position_modules;

public abstract class Encoder {

    int temp;

    abstract int getCurrentPosition();

    public int deltaPosition() {
        int a = getCurrentPosition() - temp;
        temp = getCurrentPosition();
        return a;
    }
}
