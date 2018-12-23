package org.firstinspires.ftc.teamcode.Distance_Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.RoverRuckusMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.drivetrain_test.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;




/**
 * Created by DrZ on 12/19/18.
 */
@TeleOp(name="Corrdinate Testing")
public class Corrdinate_Test extends  LinearOpMode {
    private DistanceSensor sensorrange0;
    private DistanceSensor sensorrange1;
    private DistanceSensor sensorrange2;
    private DistanceSensor sensorrange3;
    double front;
    double right;
    double left;
    double back;
    double x = 0;
    double y = 0;
    double fieldsize = 182.88;
    boolean turningTowards = false;
    final double turnSpeed = 0.6;
    final double slowSpeed = 0.4; // 0.8


   

    public void runOpMode() {

        RobotHardware rw = new RobotHardware(hardwareMap);

        RoverRuckusMecanumDriveREVOptimized drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap);

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(rw);
        drivetrain = new FourWheelMecanumDrivetrain(rw);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setSpeedMultiplier(slowSpeed);



        sensorrange0 = hardwareMap.get(DistanceSensor.class, "leftsens");
        sensorrange1 = hardwareMap.get(DistanceSensor.class, "frontsens");
        sensorrange2 = hardwareMap.get(DistanceSensor.class, "backsens");
        sensorrange3 = hardwareMap.get(DistanceSensor.class, "rightsens");
        telemetry.addData("left", sensorrange0.getDistance(DistanceUnit.CM));
        telemetry.addData("front", sensorrange1.getDistance(DistanceUnit.CM));
        telemetry.addData("back", sensorrange2.getDistance(DistanceUnit.CM));
        telemetry.addData("Right", sensorrange3.getDistance(DistanceUnit.CM));
        telemetry.addData("Ready", "ready");
        telemetry.addData("Heading", radToDeg(drive.getExternalHeading()));
        telemetry.update();

        front = sensorrange1.getDistance(DistanceUnit.CM);
        back = sensorrange2.getDistance(DistanceUnit.CM);
        right = sensorrange3.getDistance((DistanceUnit.CM));
        left = sensorrange0.getDistance(DistanceUnit.CM);
        waitForStart();

        while (opModeIsActive()) {
            double Vertical = Math.min(front, back);
            double Horizontal = Math.min(left,right);
            double Heading = radToDeg(drive.getExternalHeading());
            // Driving Gamepads logic
            // region driving
            double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0) && !turningTowards) {

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
                if (Heading > 0 && Heading < 45) {
                    x = fieldsize  - Math.cos(Math.toRadians(Heading)) * Vertical;
                    y = fieldsize  - Math.cos(Math.toRadians(Heading)) * Horizontal;
                } else if (Heading > 45 && Heading < 90) {
                    x = fieldsize  - Math.cos(Math.toRadians(90 - Heading)) * Horizontal;
                    y = fieldsize  - Math.cos(Math.toRadians(90 - Heading)) * Vertical; 
                } else if (Heading > 90 && Heading < 135) {
                    x = fieldsize  - Math.cos(Math.toRadians(Heading - 90)) * Horizontal;
                    y = fieldsize  - Math.cos(Math.toRadians(Heading - 90)) * Vertical; 
                } else if (Heading > 135 && Heading < 180) {
                    x = fieldsize  - Math.cos(Math.toRadians(180 -Heading)) * Vertical; 
                    y = fieldsize  - Math.cos(Math.toRadians(180 -Heading)) * Horizontal;
                } else if (Heading < 225 && Heading > 180) {
                    x = fieldsize  - Math.cos(Math.toRadians(Heading - 180)) * Vertical; 
                    y = fieldsize  - Math.cos(Math.toRadians(Heading - 180)) * Horizontal;
                } else if (Heading < 270 && Heading > 225) {
                    x = fieldsize  - Math.cos(Math.toRadians(270 - Heading)) * Horizontal;
                    y = fieldsize  - Math.cos(Math.toRadians(270 - Heading)) * Vertical; 
                } else if (Heading < 315 && Heading > 270) {
                    x = fieldsize  - Math.cos(Math.toRadians(Heading - 270)) * Horizontal;
                    y = fieldsize  - Math.cos(Math.toRadians(Heading - 270)) * Vertical;
                } else if (Heading < 360 && Heading > 315) {
                    x = fieldsize  - Math.cos(Math.toRadians(360 - Heading)) * Vertical;
                    y = fieldsize  - Math.cos(Math.toRadians(360 - Heading)) * Horizontal;
                }else if(Heading == 0 || Heading == 180 || Heading == 360){
                    x = fieldsize  - Vertical; 
                    y = fieldsize  - Horizontal;
                }else if(Heading == 90 || Heading == 270){
                    x = fieldsize  - Horizontal;
                    y = fieldsize  - Vertical; 
                }
                if(Math.min(front, back)==front&&Math.min(right, left)==right){
                    x = 0-x;
                }
                if(Math.min(front, back)==front&&Math.min(right, left)== left){
                    x = 0-x;
                    y = 0-y;
                }
                if(Math.min(front, back)== back &&Math.min(right, left)== left){
                    y = 0-y;
                }
                telemetry.addData("X:", x);
                telemetry.addData("Y",y);
                telemetry.addData("degree",Heading);
            telemetry.addData("left", sensorrange0.getDistance(DistanceUnit.CM));
            telemetry.addData("front", sensorrange1.getDistance(DistanceUnit.CM));
            telemetry.addData("back", sensorrange2.getDistance(DistanceUnit.CM));
            telemetry.addData("Right", sensorrange3.getDistance(DistanceUnit.CM));

                telemetry.update();

            }


            }
    double radToDeg(double rad) {
        return rad * 180 / Math.PI;
    }



}


