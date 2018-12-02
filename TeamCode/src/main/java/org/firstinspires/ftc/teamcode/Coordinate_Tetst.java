package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.GyroSensor;
import java.lang.String;

/**
 * Created by DrZ on 11/30/18.
 */

public class Coordinate_Tetst {
    double fake = 49;
    static double x;
    static double y;
    double a = 32.31;
    double b = 60.64;
    double c = 72.20;
    double d = 23.21;

    public void calcDistance1() {

        if (fake > 0 && fake < 45) {
            x = 182.88 - Math.cos(Math.toRadians(45 - fake)) * a;
            y = 182.88 - Math.cos(45 - fake) * b;
        } else if (fake > 45 && fake < 90) {
            x = 182.88 - Math.cos(fake - 45) * a;
            y = 182.88 - Math.cos(fake - 45) * c;
        } else if (fake > 90 && fake < 135) {
            x = 182.88 - Math.cos(135 - fake) * b;
            y = 182.88 - Math.cos(135 - fake) * a;
        } else if (fake > 135 && fake < 180) {
            x = 182.88 - Math.cos(fake - 135) * c;
            y = 182.88 - Math.cos(fake - 135) * a;
        } else if (fake < 225 && fake > 180) {
            x = 182.88 - Math.cos(225 - fake) * a;
            y = 182.88 - Math.cos(225 - fake) * b;
        } else if (fake < 270 && fake > 225) {
            x = 182.88 - Math.cos(fake - 225) * a;
            y = 182.88 - Math.cos(fake - 225) * b;
        } else if (fake < 315 && fake > 270) {
            x = 182.88 - Math.cos(315 - fake) * b;
            y = 182.88 - Math.cos(315 - fake) * a;
        } else if (fake < 360 && fake > 315) {
            x = 182.88 - Math.cos(fake - 315) * c;
            y = 182.88 - Math.cos(fake - 315) * a;
        } else if (fake < 0 && fake > -45) {
            x = 182.88 - Math.cos(45 - Math.abs(fake)) * c;
            y = 182.88 - Math.cos(45 -Math.abs(fake)) * a;

        }else if (fake < -45 && fake > -90) {
            x = 182.88 - Math.cos(Math.abs(fake) - 45) * b;
            y = 182.88 - Math.cos(Math.abs(fake) - 45) * a;

        }else if (fake < -90 && fake > -135) {
            x = 182.88 - Math.cos(135 - Math.abs(fake)) * a;
            y = 182.88 - Math.cos(135 - Math.abs(fake)) * c;

        }else if (fake < -135 && fake > -180) {
            x = 182.88 - Math.cos(Math.abs(fake) - 135) * a;
            y = 182.88 - Math.cos(Math.abs(fake) - 135) * b;

        }else if (fake < -180 && fake > -225) {
            x = 182.88 - Math.cos(225 - Math.abs(fake)) * c;
            y = 182.88 - Math.cos(225 - Math.abs(fake)) * a;

        }else if (fake < -225 && fake > -270) {
            x = 182.88 - Math.cos(Math.abs(fake) - 225) * b;
            y = 182.88 - Math.cos(Math.abs(fake) - 225) * a;

        }else if (fake < -270 && fake > -315) {
            x = 182.88 - Math.cos(315 - Math.abs(fake)) * a;
            y = 182.88 - Math.cos(315 - Math.abs(fake)) * c;

        }else if (fake < -315 && fake > -360) {
            x = 182.88 - Math.cos(fake - 315) * a;
            y = 182.88 - Math.cos(fake - 315) * b;

        }


    }
    public static void main(String args[]){
        System.out.println("x = " + x + "y= " + y);
    }

}

   