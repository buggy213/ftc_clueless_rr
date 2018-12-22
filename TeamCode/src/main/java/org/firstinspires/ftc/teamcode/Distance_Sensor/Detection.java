package org.firstinspires.ftc.teamcode.Distance_Sensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drivetrain_test.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.drivetrain_test.RobotHardware;

/**
 * Created by DrZ on 12/18/18.
 */
//@TeleOp(name="Distance Detection")//TODO: increase the value of abs(y-x), greater than 10)
public class Detection extends LinearOpMode {
    double x;
    double y = 0;
    boolean blocked;
    DistanceSensor sensorRange1;


    public void runOpMode() {

        x = sensorRange1.getDistance(DistanceUnit.CM);
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "backsens");
        telemetry.addData(">>", "Press start to continue");
        telemetry.addData("range", String.format("%.01f cm", sensorRange1.getDistance(DistanceUnit.CM)));
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            if(Math.abs(y - x) > 50){
                blocked = true;
            }else{
                blocked = false;
            }
            y = sensorRange1.getDistance(DistanceUnit.CM);

            while(blocked = true){//Todo: question mark? on x,y switch
                x = y;
                y = sensorRange1.getDistance(DistanceUnit.CM);
                if(Math.abs(y -x) > 50){
                    blocked = false;
                    break;
                }else{
                    blocked = true;
                }
                telemetry.addData("blocked?",blocked);
                telemetry.addData("idk", Math.abs(y - x));
                telemetry.update();
            }
        }
        }
    }



