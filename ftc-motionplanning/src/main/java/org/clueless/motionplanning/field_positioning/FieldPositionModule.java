package org.clueless.motionplanning.field_positioning;

import org.clueless.motionplanning.math.TwoDimensionalTransform;
import org.clueless.motionplanning.math.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class FieldPositionModule {
    protected TwoDimensionalTransform transform;

    public void SetReferencePosition(TwoDimensionalTransform referencePosition) {
        transform = referencePosition;
    }

    public abstract TwoDimensionalTransform Update();

    /**
     * Updates robot position based on x + y components as well as a change in the angle
     * @param deltaX Horizontal movement (in mm)
     * @param deltaY Vertical movement (in mm)
     * @param deltaAngle Change in angle
     */

    public void AddVector(double deltaX, double deltaY, double deltaAngle) {
        Vector2 forwardVector = new Vector2(Math.cos(transform.angle), Math.sin(transform.angle));
        Vector2 rightVector = new Vector2(Math.cos(transform.angle + Math.PI / 4), Math.sin(transform.angle + Math.PI / 4));

        Vector2 combined = Vector2.add(forwardVector.multiply(deltaY), rightVector.multiply(deltaX));
        transform.translate(combined);
        transform.rotate(deltaAngle);
    }
}
