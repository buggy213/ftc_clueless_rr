
package org.firstinspires.ftc.teamcode;

import android.util.Log;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
@Config
@Autonomous
public class Vuforia_Test extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";
    OpenGLMatrix lastLocation = null;
    boolean inRange = false;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    //TODO Change X,Y,Z constraints to whatever is necessary.
    float targetXMin = -10;
    float targetXMax = 10;
    float targetYMin = 50;
    float targetYMax = 60;
    float targetZMin = -10;
    float targetZMax = 10;
    float currentX = 0;
    float currentY = 0;
    float currentZ = 0;
    WebcamName webcamName;
    @Override public void runOpMode() {
        waitForStart();
        vuforiaSearch();
    }
    private void vuforiaSearch() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";  //Variable Place--Remember to insert key here
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);  //Instantiate the Vuforia engine
        vuforia.enableConvertFrameToBitmap();
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");  //Getting picture data
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
        Log.d("Debug: ", "Before OpenGLMatrix!!");
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueRover.setLocationFtcFieldFromTarget(blueRoverLocationOnField);
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redFootprint.setLocationFtcFieldFromTarget(redFootprintLocationOnField);
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        frontCraters.setLocationFtcFieldFromTarget(frontCratersLocationOnField);
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        backSpace.setLocationFtcFieldFromTarget(backSpaceLocationOnField);
        final int CAMERA_FORWARD_DISPLACEMENT = 218;   // Camera is 217.5 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 120;   // Camera is 120 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 110;   // Camera is 110 mm off of the center line
        OpenGLMatrix camLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
        Log.d("Debug: ", "Passed OpenGLMatrix!!");
        Log.d("Debug: ", "Before VuforiaTrackableDefaultListener!!");
        ((VuforiaTrackableDefaultListener) blueRover.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        ((VuforiaTrackableDefaultListener) redFootprint.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        ((VuforiaTrackableDefaultListener) frontCraters.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        ((VuforiaTrackableDefaultListener) backSpace.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        Log.d("Debug: ", "After VuforiaTrackableDefaultListener!!");
        /*targetsRoverRuckus.activate();
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));
        Log.d("Debug: ", "Passed OpenGLMatrix!!");
        Log.d("Debug: ", "Before VuforiaTrackableDefaultListener!!");
        ((VuforiaTrackableDefaultListener) blueRover.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) redFootprint.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) frontCraters.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) backSpace.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        Log.d("Debug: ", "After VuforiaTrackableDefaultListener!!");*/
        waitForStart();
        targetsRoverRuckus.activate();

        while (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            if (lastLocation != null) {
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                currentX = translation.get(0) / mmPerInch;
                currentY = translation.get(1) / mmPerInch;
                currentZ = translation.get(2) / mmPerInch;
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                //TODO Execute some commands if the location is within X,Y,Z constraints.
                if (currentX >= targetXMin && currentX <= targetXMax && currentY >= targetYMin && currentY <= targetYMax && currentZ >= targetZMin && currentZ <= targetZMax) {
                    telemetry.addData("Target Status: ", "You have reached your target.");
                } else {
                    telemetry.addData("Target Status: ", "You have not reached your target.");
                    currentX = translation.get(0) / mmPerInch;
                    currentY = translation.get(1) / mmPerInch;
                    currentZ = translation.get(2) / mmPerInch;
                }
                // express the rotation of the robot in degrees.
            } else {
                telemetry.addData("LastLocation = ", "null");
            }
            if (lastLocation != null) {
                telemetry.addData("Robot Field Coordinates: ", "currentX = " + currentX + ", currentY = " + currentY + ", currentZ = " + currentZ);
                inRange = true;
            } else {
                inRange = false;
                telemetry.addData("Robot Field Coordinates: ", "Coordinates are Unknown");
                telemetry.addData("Pos", "currentX = " + currentX + ", currentY = " + currentY + ", currentZ = " + currentZ);
                inRange = true;
            }
            telemetry.update();
        }
    }
    public void main(String[] args){

    }

}
