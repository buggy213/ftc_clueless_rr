package org.firstinspires.ftc.teamcode.test.distance_sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;

/**
 * Created by DrZ on 11/23/18.
 */

@TeleOp(name="Distance Test?")
public class DistanceSensorTest extends LinearOpMode {
    private DistanceSensor sensorRange1;
    private DistanceSensor sensorRange2;
    private DistanceSensor sensorRange3;
    private DistanceSensor sensorRange4;
    double a;
    double b;
    double c;
    double d;
    double x;
    double y;
    boolean blocked;
    GyroSensor g;




    @Override
    public void runOpMode() {
        RobotHardware rw = new RobotHardware(hardwareMap);

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(rw);



        // you can use this as a regular DistanceSensor.
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "Ds1");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "Ds2");
        sensorRange3 = hardwareMap.get(DistanceSensor.class, "Ds3");
        sensorRange4 = hardwareMap.get(DistanceSensor.class, "Ds4");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange1;
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor)sensorRange2;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor)sensorRange3;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor)sensorRange4;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger);

            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0)) {

                double speed;

                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    speed = 0;
                }
                else if ( gamepad1.right_stick_y == 0 ) {
                    speed = Math.abs(gamepad1.left_stick_x) ;
                }
                else if ( gamepad1.left_stick_x == 0 ) {
                    speed = Math.abs(gamepad1.right_stick_y) ;
                }
                else {
                    speed = ( Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y) ) / 2;
                }

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                drivetrain.MoveAngle(speed, angle, turn);
            } else {
                drivetrain.stop();
            }
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



    public void calcDis1() {//red depo

        if (g.getHeading() > 0 && g.getHeading() < 45) {
            x = 182.88 - Math.cos(Math.toRadians(45 - g.getHeading())) * Math.min(a,d);
            y = 182.88 - Math.cos(Math.toRadians(45 - g.getHeading())) * Math.min(b,c);
        } else if (g.getHeading() > 45 && g.getHeading() < 90) {
            x = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 45)) * Math.min(b, c);
            y = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 45)) * Math.min(a, d);
        } else if (g.getHeading() > 90 && g.getHeading() < 135) {
            x = 182.88 - Math.cos(Math.toRadians(135 - g.getHeading())) * Math.min(b, c);
            y = 182.88 - Math.cos(Math.toRadians(135 - g.getHeading())) * Math.min(a, d);
        } else if (g.getHeading() > 135 && g.getHeading() < 180) {
            x = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 135)) * Math.min(a, d);
            y = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 135)) * Math.min(b, c);
        } else if (g.getHeading() < 225 && g.getHeading() > 180) {
            x = 182.88 - Math.cos(Math.toRadians(225 - g.getHeading())) * Math.min(a, d);
            y = 182.88 - Math.cos(Math.toRadians(225 - g.getHeading())) * Math.min(b, c);
        } else if (g.getHeading() < 270 && g.getHeading() > 225) {
            x = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 225)) * Math.min(b, c);
            y = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 225)) * Math.min(a, d);
        } else if (g.getHeading() < 315 && g.getHeading() > 270) {
            x = 182.88 - Math.cos(Math.toRadians(315 - g.getHeading())) * Math.min(b,c);
            y = 182.88 - Math.cos(Math.toRadians(315 - g.getHeading())) * Math.min(a,d);
        } else if (g.getHeading() < 360 && g.getHeading() > 315) {
            x = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 315)) * Math.min(a, d);
            y = 182.88 - Math.cos(Math.toRadians(g.getHeading() - 315)) * Math.min(b, c);
        }else if(g.getHeading() == 0 || g.getHeading() == 180 || g.getHeading() == 360){
            x = 182.88 - Math.min(a,d);
            y = 182.88 - Math.min(b,c);
        }else if(g.getHeading() == 90 || g.getHeading() == 270){
            x = 182.88 - Math.min(b,c);
            y = 182.88 - Math.min(a,d);
        }


    }


public void calcQ(){
        if(Math.min(a,d)==a&&Math.min(b,c)==b){
            x = 0-x;
        }
        if(Math.min(a,d)==a&&Math.min(b,c)==c){
            x = 0-x;
            y = 0-y;
        }
        if(Math.min(a,d)==d&&Math.min(b,c)==c){
            y = 0-y;
        }
}

}








