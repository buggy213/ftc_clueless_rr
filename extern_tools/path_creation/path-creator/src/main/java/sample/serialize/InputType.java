package sample.serialize;

import com.acmerobotics.roadrunner.Pose2d;
import jdk.internal.util.xml.impl.Input;

public enum InputType {
    BOOLEAN,
    STRING,
    NUMBER,
    POSITION;

    public static Object getDefault(InputType inputType) {
        switch (inputType) {
            case BOOLEAN:
                return false;
            case STRING:
                return "";
            case NUMBER:
                return 0;
            case POSITION:
                return new Pose2d(0,0,0);
            default:
                return null;
        }
    }
}