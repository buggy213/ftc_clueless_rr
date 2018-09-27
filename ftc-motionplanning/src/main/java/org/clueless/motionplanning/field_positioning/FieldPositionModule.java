package org.clueless.motionplanning.field_positioning;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public abstract class FieldPositionModule {
    protected OpenGLMatrix pos;

    public void SetReferencePosition(OpenGLMatrix referencePosition) {
        pos = referencePosition;
    }

    public abstract OpenGLMatrix Update();

}
