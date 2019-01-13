package org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.io.FileUtils;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autonomous.parameters.Parameters;
import org.firstinspires.ftc.teamcode.autonomous.parameters.SelectParameters;
import org.firstinspires.ftc.teamcode.motionplanning.util.DashboardUtil;

import java.io.File;
import java.io.FileFilter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.charset.Charset;

@TeleOp(name="Trajectory Viewer")
public class TrajectoryViewerOpMode extends LinearOpMode {

    private ObjectMapper objectMapper = new ObjectMapper();

    private int index;
    private boolean a, b;

    private Trajectory t;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        File[] trjFiles = AppUtil.FIRST_FOLDER.listFiles((File f, String s) -> s.endsWith(".trj"));

        waitForStart();

        while (opModeIsActive()) {
            if (!gamepad1.a && a) {
                index++;
                loadAndDisplayTrajectory(trjFiles[index]);
            }
            if (!gamepad1.b && b) {
                index--;
                loadAndDisplayTrajectory(trjFiles[index]);
            }

            if (t != null) {
                TelemetryPacket packet = new TelemetryPacket();
                DashboardUtil.drawSampledTrajectory(packet.fieldOverlay(), t, 250);
                dashboard.sendTelemetryPacket(packet);
            }

            a = gamepad1.a;
            b = gamepad2.b;

            index = ((index % trjFiles.length + trjFiles.length) % trjFiles.length);
        }
    }

    private void loadAndDisplayTrajectory(File file) {
        try {
            TrajectoryBuilderWrapper tbw = objectMapper.readValue(file, TrajectoryBuilderWrapper.class);
            telemetry.addData("Name", tbw.name);
            t = tbw.toTrajectoryBuilder().build();

        }
        catch(IOException e) {
            telemetry.addData("Couldn't open trajectory", file.getName());
            telemetry.update();
        }
    }
}
