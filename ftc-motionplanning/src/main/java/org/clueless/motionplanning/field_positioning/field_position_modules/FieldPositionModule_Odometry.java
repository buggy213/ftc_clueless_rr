package org.clueless.motionplanning.field_positioning.field_position_modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.clueless.motionplanning.field_positioning.FieldPositionModule;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public class FieldPositionModule_Odometry extends FieldPositionModule {

    AndroidGyroscope androidGyroscope;
    Gyroscope gyroscope;





    @Override
    public OpenGLMatrix Update() {
        // Position should never be null -- there needs to be a starting point for odometry integration to make sense
        if (pos == null) {
            throw new RuntimeException();
        }

        return null;
    }
}
