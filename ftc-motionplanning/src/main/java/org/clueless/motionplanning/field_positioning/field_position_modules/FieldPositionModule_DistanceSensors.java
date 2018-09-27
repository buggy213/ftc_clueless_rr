package org.clueless.motionplanning.field_positioning.field_position_modules;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.clueless.motionplanning.field_positioning.FieldPositionModule;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.List;

public class FieldPositionModule_DistanceSensors extends FieldPositionModule{

    List<DistanceSensor> distanceSensors;
    List<OpenGLMatrix> distanceSensorPositions;

    @Override
    public OpenGLMatrix Update() {
        // 1. Determine current orientation on field using gyroscopes (also potentially data from odometry wheels if they turn out to be more reliable)
        // 2. 
        return null;
    }
}
