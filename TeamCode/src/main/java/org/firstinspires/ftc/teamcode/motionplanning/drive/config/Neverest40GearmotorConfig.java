package org.firstinspires.ftc.teamcode.motionplanning.drive.config;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;

public class Neverest40GearmotorConfig implements GearmotorConfig{
    @Override
    public int getMaxRPM() {
        return 160;
    }

    @Override
    public int getTicksPerRev() {
        return 1120;
    }

}
