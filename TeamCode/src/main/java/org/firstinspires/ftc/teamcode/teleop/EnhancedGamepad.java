package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;
import java.util.Map;

public class EnhancedGamepad {
    private Gamepad previousGamepad;
    private Gamepad currentGamepad;

    private Map<String, Boolean> toggles = new HashMap<>();

    public void update(Gamepad g) {
        if (currentGamepad != null) {
            try {
                previousGamepad.copy(currentGamepad);
            }
            catch(RobotCoreException e) {
                RobotLog.e(e.toString());
            }
        }
        currentGamepad = g;
    }

    private boolean getButton(String button, Gamepad g) {
        return g.toString().contains(button);
    }

    public boolean getButton(String button) {
        return getButton(button, currentGamepad);
    }

    public boolean getTrigger(String button) {
        if (!toggles.containsKey(button)) {
            toggles.put(button, false);
        }
        if (currentGamepad != null && previousGamepad != null) {
            if (getButton(button, currentGamepad) && !getButton(button, previousGamepad)) {
                toggles.put(button, !toggles.get(button));
            }
        }

        return toggles.get(button);
    }
}
