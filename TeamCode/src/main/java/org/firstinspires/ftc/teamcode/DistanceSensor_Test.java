package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by DrZ on 11/23/18.
 */

public class DistanceSensor_Test extends LinearOpMode {
    private DistanceSensor sensorRange1;
    private DistanceSensor sensorRange2;
    private DistanceSensor sensorRange3;
    private DistanceSensor sensorRange4;
    double distanceS1;
    double distanceS2;
    double distanceS3;
    double distanceS4;
    double a;
    double b;
    double c;
    double d;
    double x;
    double y;
    GyroSensor g;
    Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) sensorRange1;
    Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorRange2;
    Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor) sensorRange3;
    Rev2mDistanceSensor sensorTimeOfFlight4 = (Rev2mDistanceSensor) sensorRange4;



    ;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "Ds1");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "Ds2");
        sensorRange3 = hardwareMap.get(DistanceSensor.class, "Ds3");
        sensorRange4 = hardwareMap.get(DistanceSensor.class, "Ds4");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange1;
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor)sensorRange1;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor)sensorRange1;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor)sensorRange1;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.

            telemetry.addData("range", String.format("%.01f cm", sensorRange1.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange2.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange3.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange4.getDistance(DistanceUnit.CM)));


            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight1.getModelID()));
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight2.getModelID()));
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight3.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight1.didTimeoutOccur()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight2.didTimeoutOccur()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight3.didTimeoutOccur()));

            telemetry.update();
        }
    }


    /*
    public void calcDistance1() {

        if (g.getHeading() > 0 && g.getHeading() < 45) {
            x = 182.88 - Math.cos(45 - g.getHeading()) * a;
            y = 182.88 - Math.cos(45 - g.getHeading()) * b;
        } else if (g.getHeading() > 45 && g.getHeading() < 90) {
            x = 182.88 - Math.cos(g.getHeading() - 45) * a;
            y = 182.88 - Math.cos(g.getHeading() - 45) * c;
        } else if (g.getHeading() > 90 && g.getHeading() < 135) {
            x = 182.88 - Math.cos(135 - g.getHeading()) * b;
            y = 182.88 - Math.cos(135 - g.getHeading()) * a;
        } else if (g.getHeading() > 135 && g.getHeading() < 180) {
            x = 182.88 - Math.cos(g.getHeading() - 135) * c;
            y = 182.88 - Math.cos(g.getHeading() - 135) * a;
        } else if (g.getHeading() < 225 && g.getHeading() > 180) {
            x = 182.88 - Math.cos(225 - g.getHeading()) * a;
            y = 182.88 - Math.cos(225 - g.getHeading()) * b;
        } else if (g.getHeading() < 270 && g.getHeading() > 225) {
            x = 182.88 - Math.cos(g.getHeading() - 225) * a;
            y = 182.88 - Math.cos(g.getHeading() - 225) * b;
        } else if (g.getHeading() < 315 && g.getHeading() > 270) {
            x = 182.88 - Math.cos(315 - g.getHeading()) * b;
            y = 182.88 - Math.cos(315 - g.getHeading()) * a;
        } else if (g.getHeading() < 360 && g.getHeading() > 315) {
            x = 182.88 - Math.cos(g.getHeading() - 315) * c;
            y = 182.88 - Math.cos(g.getHeading() - 315) * a;
        } else if (g.getHeading() < 0 && g.getHeading() > -45) {
            x = 182.88 - Math.cos(45 - Math.abs(g.getHeading())) * c;
            y = 182.88 - Math.cos(45 -Math.abs(g.getHeading())) * a;

        }else if (g.getHeading() < -45 && g.getHeading() > -90) {
            x = 182.88 - Math.cos(Math.abs(g.getHeading()) - 45) * b;
            y = 182.88 - Math.cos(Math.abs(g.getHeading()) - 45) * a;

        }else if (g.getHeading() < -90 && g.getHeading() > -135) {
            x = 182.88 - Math.cos(135 - Math.abs(g.getHeading())) * a;
            y = 182.88 - Math.cos(135 - Math.abs(g.getHeading())) * c;

        }else if (g.getHeading() < -135 && g.getHeading() > -180) {
            x = 182.88 - Math.cos(Math.abs(g.getHeading()) - 135) * a;
            y = 182.88 - Math.cos(Math.abs(g.getHeading()) - 135) * b;

        }else if (g.getHeading() < -180 && g.getHeading() > -225) {
            x = 182.88 - Math.cos(225 - Math.abs(g.getHeading())) * c;
            y = 182.88 - Math.cos(225 - Math.abs(g.getHeading())) * a;

        }else if (g.getHeading() < -225 && g.getHeading() > -270) {
            x = 182.88 - Math.cos(Math.abs(g.getHeading()) - 225) * b;
            y = 182.88 - Math.cos(Math.abs(g.getHeading()) - 225) * a;

        }else if (g.getHeading() < -270 && g.getHeading() > -315) {
            x = 182.88 - Math.cos(315 - Math.abs(g.getHeading())) * a;
            y = 182.88 - Math.cos(315 - Math.abs(g.getHeading())) * c;

        }else if (g.getHeading() < -315 && g.getHeading() > -360) {
            x = 182.88 - Math.cos(g.getHeading() - 315) * a;
            y = 182.88 - Math.cos(g.getHeading() - 315) * b;

        }


        }
*/
    public void main(String[] args) {

    }
}








