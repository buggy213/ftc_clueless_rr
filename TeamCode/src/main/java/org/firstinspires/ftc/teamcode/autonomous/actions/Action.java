package org.firstinspires.ftc.teamcode.autonomous.actions;

interface Action extends Runnable {
    // Start watching for conditions to start action
    void watch();
    // Start action
    void start();
    // Update while action isn't finished
    void update();
    // Stop action
    void stop();
}
