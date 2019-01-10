/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.ArmSetpoints;
import org.firstinspires.ftc.teamcode.shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.shared.RobotConstants;
import org.firstinspires.ftc.teamcode.shared.RobotHardware;
import org.firstinspires.ftc.teamcode.shared.RoverRuckusMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.shared.Vuforia;

import java.util.List;

import static org.firstinspires.ftc.teamcode.shared.RobotConstants.DOOR_BLOCK;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.DOOR_RELEASED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_COLLECT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_JOINT_UP;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.INTAKE_SPEED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.LOCK_DISENGAGED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.LOCK_ENGAGED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.NARROW_CUBE_CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.NARROW_CUBE_CLAW_RIGHT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.SORTER_OUT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.SORTER_TUCKED;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.WIDE_CLAW_LEFT;
import static org.firstinspires.ftc.teamcode.shared.RobotConstants.WIDE_CLAW_RIGHT;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="TeleOp", group="Linear Opmode")
public class TelemetryOpmode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Constants for teleop
    final double turnSpeed = 0.6;
    final double slowSpeed = 0.8; // 0.8

    boolean turningTowards = false;
    boolean manualArmControl = true;

    boolean door;
    boolean doorToggle;

    int intakeMode = 1;

    double previousTimeStamp = 0;

    private FourWheelMecanumDrivetrain drivetrain;
    private ArmController armController;
    private Vuforia vuforia;
    private RoverRuckusMecanumDriveREVOptimized drive;

    public static boolean debugPosition;

    private boolean movingToSetpoint;

    @Override
    public void runOpMode() {
        RobotHardware rw = new RobotHardware(hardwareMap);

        if (debugPosition) {
            drive = new RoverRuckusMecanumDriveREVOptimized(hardwareMap);
            vuforia = new Vuforia();
            vuforia.init(hardwareMap);
        }

        boolean back = false;

        armController = new ArmController(rw, telemetry, true);
        if (manualArmControl) armController.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain = new FourWheelMecanumDrivetrain(rw);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain.setSpeedMultiplier(slowSpeed);
        drivetrain.resetEncoders();

        // drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rw.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (debugPosition) {
            vuforia.activateTargets();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double deltaTime = runtime.milliseconds() - previousTimeStamp;

            if (debugPosition) {
                drive.updatePoseEstimate();
                Pose2d odometryEstimate = drive.getPoseEstimate();
                Pose2d vuforiaEstimate = vuforia.getPosition();
                TelemetryPacket packet = new TelemetryPacket();
                Canvas canvas = packet.fieldOverlay();
                canvas.setFill("green");
                canvas.fillCircle(odometryEstimate.getX(), odometryEstimate.getY(), 2.5);
                canvas.setFill("blue");
                canvas.fillCircle(vuforiaEstimate.getX(), vuforiaEstimate.getY(), 2.5);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            // Driving Gamepads logic
            // region driving
            double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0) && !turningTowards) {

                double speed;

                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    speed = 0;
                }
                else if ( gamepad1.right_stick_y == 0 ) {
                    speed = Math.sqrt(2) * Math.abs(gamepad1.left_stick_x) ;
                }
                else if ( gamepad1.left_stick_x == 0 ) {
                    speed = Math.abs(gamepad1.right_stick_y) ;
                }
                else {
                    speed = Math.min( Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y), 1);
                }

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                drivetrain.MoveAngle(speed, angle, turn, false);
            } else {
                drivetrain.stop();
            }

            telemetry.addData("FR", drivetrain.rw.frontRight.getCurrentPosition());

            telemetry.addData("FL", drivetrain.rw.frontLeft.getCurrentPosition());

            telemetry.addData("BR", drivetrain.rw.backRight.getCurrentPosition());

            telemetry.addData("BL", drivetrain.rw.backLeft.getCurrentPosition());

            telemetry.addData("Left stick x", gamepad1.left_stick_x);
            telemetry.addData("Right stick y", gamepad1.right_stick_y);

            telemetry.addData("Linear Slider Position", rw.linearSlider.getCurrentPosition());

            //endregion
            if (gamepad2.b) {
                rw.pawServo.setPosition(LOCK_ENGAGED);
            }
            if (gamepad2.a) {
                rw.pawServo.setPosition(LOCK_DISENGAGED);
            }

            if (gamepad2.back && !back) {
                manualArmControl = !manualArmControl;
            }

            if (gamepad1.x) {
                rw.intakeJoint.setPosition(INTAKE_JOINT_COLLECT);
            }

            if (gamepad1.y) {
                rw.intakeJoint.setPosition(INTAKE_JOINT_UP);
            }

            door = gamepad1.b;
            back = gamepad2.back;

            rw.linearSlider.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


            if (gamepad2.left_bumper) {
                movingToSetpoint = true;
                armController.setPositions(ArmSetpoints.SCORE);
            }

            if (gamepad2.right_bumper) {
                movingToSetpoint = true;
                armController.setPositions(ArmSetpoints.COLLECT);
            }

            if (gamepad2.dpad_left) {
                rw.leftClaw.setPosition(RobotConstants.OPEN_CLAW_LEFT);
                rw.rightClaw.setPosition(RobotConstants.OPEN_CLAW_RIGHT);
            }

            if (gamepad2.dpad_right) {
                rw.leftClaw.setPosition(RobotConstants.WIDE_CLAW_LEFT);
                rw.rightClaw.setPosition(RobotConstants.WIDE_CLAW_RIGHT);
            }

            if (gamepad2.dpad_up) {
                rw.leftClaw.setPosition(NARROW_CUBE_CLAW_LEFT);
                rw.rightClaw.setPosition(WIDE_CLAW_RIGHT);
            }

            if (gamepad2.dpad_down) {
                rw.leftClaw.setPosition(WIDE_CLAW_LEFT);
                rw.rightClaw.setPosition(NARROW_CUBE_CLAW_RIGHT);
            }

            if (gamepad1.dpad_up) {
                rw.leftClaw.setPosition(RobotConstants.NARROW_CUBE_CLAW_LEFT);
                rw.rightClaw.setPosition(RobotConstants.NARROW_CUBE_CLAW_RIGHT);
            }

            if (movingToSetpoint) {
                armController.updateArmAuto();
                if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                    movingToSetpoint = false;
                }
            }
            else if (manualArmControl) {
                armController.manualArmControl(gamepad2);
            }
            else {
                armController.basicKinematicControl(gamepad2);
            }
            telemetry.update();

            previousTimeStamp = runtime.milliseconds();
        }
    }


}