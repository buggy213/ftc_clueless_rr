package org.clueless.motionplanning.field_positioning.field_position_modules.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.clueless.motionplanning.field_positioning.FieldPositionModule;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;


//

public  abstract class FieldPositionModule_Odometry extends FieldPositionModule {

    OdometryWheelProperties wheelProperties;

    public class OdometryWheelProperties {
        int encodersPerRevolution;
        double wheelDiameter; // In millimeters
    }

    public void setWheelProperties(OdometryWheelProperties wheelProperties) {
        this.wheelProperties = wheelProperties;
    }


}
