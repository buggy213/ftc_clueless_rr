package org.firstinspires.ftc.teamcode.motionplanning.drive.config;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;

public class Neverest20GearmotorConfig implements GearmotorConfig{
    public int getTicksPerRev() {
        return 560;
    }
    public int getMaxRPM() {
        return 315;
    }

}
