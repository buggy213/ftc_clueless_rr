package org.clueless.motionplanning.field_positioning.field_position_modules.Odometry;

import org.clueless.motionplanning.field_positioning.field_position_modules.Encoder;
import org.clueless.motionplanning.math.TwoDimensionalTransform;
import org.clueless.motionplanning.math.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public class H_Layout extends FieldPositionModule_Odometry {
    Encoder left;
    Encoder right;
    Encoder horizontal;


    static double encoderDifferenceRotationFactor;

    public void setEncoderDifferenceRotationFactor(double factor) {
        encoderDifferenceRotationFactor = factor;
    }

    @Override
    public TwoDimensionalTransform Update() {
        int deltaLeft = left.deltaPosition();
        int deltaRight = right.deltaPosition();
        int deltaHorizontal = horizontal.deltaPosition();

        int averageForward = (deltaLeft + deltaRight) / 2;

        double deltaY = (averageForward / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;
        double deltaX = (deltaHorizontal / wheelProperties.encodersPerRevolution) * wheelProperties.wheelDiameter;

        double angleChange = (deltaLeft - deltaRight) * encoderDifferenceRotationFactor;

        AddVector(deltaX, deltaY, angleChange);

        return transform;
    }


}
