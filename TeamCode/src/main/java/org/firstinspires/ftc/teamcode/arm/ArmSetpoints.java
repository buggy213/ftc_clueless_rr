package org.firstinspires.ftc.teamcode.arm;

public enum ArmSetpoints {
    // TODO find arm setpoints
    COLLECT(-1150,-800),
    SCORE(-100,-1800),
    COLLECT_FAR(-2250, -1500),
    START(0, 0);

    public int firstJoint;
    public int secondJoint;

    ArmSetpoints(int firstJoint, int secondJoint) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
    }
}