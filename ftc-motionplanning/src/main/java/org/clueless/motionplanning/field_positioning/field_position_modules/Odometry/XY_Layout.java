package org.clueless.motionplanning.field_positioning.field_position_modules.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.clueless.motionplanning.field_positioning.field_position_modules.Encoder;
import org.clueless.motionplanning.math.TwoDimensionalTransform;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public class XY_Layout extends FieldPositionModule_Odometry {

    Encoder forward;
    Encoder horizontal;

    BNO055IMU gyroscope;

    @Override
    public TwoDimensionalTransform Update() {
        int deltaHorizontal = horizontal.deltaPosition();
        int deltaForward = forward.deltaPosition();

        double deltaY = (deltaForward / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;
        double deltaX = (deltaHorizontal / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;

        double deltaAngle = gyroscope.getAngularOrientation().firstAngle - transform.angle;

        AddVector(deltaX, deltaY, deltaAngle);

        return transform;
    }
}
