package org.firstinspires.ftc.teamcode.arm;

public enum ArmSetpoints {
    // TODO find arm setpoints
    COLLECT(0,0),
    SCORE(0,0);

    int firstJoint;
    int secondJoint;

    ArmSetpoints(int firstJoint, int secondJoint) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
    }
}