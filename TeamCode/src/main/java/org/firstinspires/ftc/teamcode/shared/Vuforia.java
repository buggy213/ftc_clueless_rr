package org.firstinspires.ftc.teamcode.shared;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class Vuforia {

    public static final String TAG = "Vuforia Navigation Sample";
    OpenGLMatrix lastLocation = null;
    boolean inRange = false;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    WebcamName webcamName;

    VuforiaTrackables targetsRoverRuckus;
    List<VuforiaTrackable> allTrackables;

    public void init(HardwareMap hardwareMap) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";  //Variable Place--Remember to insert key here
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);  //Instantiate the Vuforia engine
        vuforia.enableConvertFrameToBitmap();
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");  //Getting picture data
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsRoverRuckus);
        float mmPerInch = 25.4f;
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;

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

        ((VuforiaTrackableDefaultListener) blueRover.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        ((VuforiaTrackableDefaultListener) redFootprint.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        ((VuforiaTrackableDefaultListener) frontCraters.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
        ((VuforiaTrackableDefaultListener) backSpace.getListener()).setCameraLocationOnRobot(parameters.cameraName, camLocationOnRobot);
    }

    public void activateTargets() {
        targetsRoverRuckus.activate();
    }

    public void updatePosition() {
        for (VuforiaTrackable trackable : allTrackables) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
    }

    public Pose2d getPosition() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            return new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle);
        }
        return null;
    }
}
