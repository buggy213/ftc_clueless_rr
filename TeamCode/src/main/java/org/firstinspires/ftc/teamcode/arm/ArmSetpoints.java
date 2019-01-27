package org.firstinspires.ftc.teamcode.arm;

public enum ArmSetpoints {
    // TODO find arm setpoints
    COLLECT(-900,-300),
    SCORE(-200,-1800);

    int firstJoint;
    int secondJoint;

    ArmSetpoints(int firstJoint, int secondJoint) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
    }
}