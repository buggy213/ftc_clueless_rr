package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.actions.Action;

public class TimeBasedAction implements Action {
    double start, end;

    protected ElapsedTime timer;
    public TimeBasedAction(double startTime, double endTime) {
        this.start = startTime;
        this.end = endTime;
        timer = new ElapsedTime();
    }
    @Override
    public void run() {
        timer.reset();
        watch();
    }

    @Override
    public void watch() {
        while(timer.time() < start) {

        }

        start();
    }

    @Override
    public void start() {
        update();
    }

    @Override
    public void update() {
        while (timer.time() < end) {
            updateFunction();
        }
        stop();
    }

    public void updateFunction() {

    }

    @Override
    public void stop() {

    }
}
