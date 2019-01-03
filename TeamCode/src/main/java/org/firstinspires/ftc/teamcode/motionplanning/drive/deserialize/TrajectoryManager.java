package org.firstinspires.ftc.teamcode.motionplanning.drive.deserialize;


import android.content.Context;
import android.util.Base64;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.io.FileUtils;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.nio.charset.Charset;

@Config
public class TrajectoryManager {
    public static String trajectory = "";
    private static String previousTrajectory = "";
    private static ObjectMapper mapper = new ObjectMapper();

    private static TrajectoryBuilder trajectoryBuilder;

    public static TrajectoryBuilder update() {
        if (!trajectory.equals(previousTrajectory)) {
            previousTrajectory = trajectory;
            try {
                String decoded = new String(Base64.decode(trajectory, Base64.DEFAULT), "ISO-8859-1");
                TrajectoryBuilderWrapper wrapper = mapper.readValue(decoded, TrajectoryBuilderWrapper.class);
                trajectoryBuilder = wrapper.toTrajectoryBuilder();
                FileUtils.writeStringToFile(new File(AppUtil.FIRST_FOLDER, wrapper.name + ".trj"), decoded, Charset.defaultCharset());
            }
            catch (Exception e) {
                RobotLog.e(e.toString());
            }

        }
        return trajectoryBuilder;
    }

    public static TrajectoryBuilder load(String name, Pose2d poseEstimate) {
        File f = new File(AppUtil.FIRST_FOLDER, name + ".trj");
        try {
            String decoded = FileUtils.readFileToString(f, Charset.defaultCharset());
            TrajectoryBuilderWrapper wrapper = mapper.readValue(decoded, TrajectoryBuilderWrapper.class);
            trajectoryBuilder = wrapper.toTrajectoryBuilder(poseEstimate);
        }
        catch (Exception e) {
            RobotLog.e(e.toString());
        }
        return trajectoryBuilder;
    }
}
