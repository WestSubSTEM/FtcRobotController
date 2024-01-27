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

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Meet 1")
@Disabled
public class Meet1Tele extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateRuntime = new ElapsedTime();
    // input motors exactly as shown below
    MecanumDrive mecanumDrive;
    GamepadEx driverOp, liftOp;
    double liftTriggerLeft, liftTriggerRight;


    DcMotorEx motorIntake, motorLift;


    //int liftTarget = 0;
    long timer = 0;

    ButtonReader buttonLiftTop, buttonLiftLeft, buttonLiftRight, buttonLiftDown;

    Servo servoPlate, servoArm, servoPixelRotate, servoPixelFlip, servoPixelLeft, servoPixelRight;
    double pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
    double pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;

    STEMperFiConstants.STATE state = STEMperFiConstants.STATE.INTAKE;

    ButtonReader buttonStateNext, buttonStatePrevious;

    IMU imu;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        servoPixelRotate = hardwareMap.get(Servo.class, "rotate");
        servoPixelRotate.setPosition(pixelRotatePosition);
        servoPixelFlip = hardwareMap.get(Servo.class, "flip");
        servoPixelFlip.setPosition(pixelFlipPosition);
        servoPixelLeft = hardwareMap.get(Servo.class, "left");
        servoPixelFlip.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight = hardwareMap.get(Servo.class, "right");
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);

        // the extended gamepad object
        driverOp = new GamepadEx(gamepad1);
        liftOp = new GamepadEx(gamepad2);
        buttonStateNext = new ButtonReader(liftOp, GamepadKeys.Button.DPAD_UP);
        buttonStatePrevious = new ButtonReader(liftOp, GamepadKeys.Button.DPAD_DOWN);


        buttonLiftTop = new ButtonReader(liftOp, GamepadKeys.Button.Y);
        buttonLiftLeft = new ButtonReader(liftOp, GamepadKeys.Button.X);
        buttonLiftRight = new ButtonReader(liftOp, GamepadKeys.Button.B);
        buttonLiftDown = new ButtonReader(liftOp, GamepadKeys.Button.A);


        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        /*
        Original flat up
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
         */

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        Motor in_e = new Motor(hardwareMap, "drive_in_e", Motor.GoBILDA.RPM_312);
        Motor in_c = new Motor(hardwareMap, "drive_in_c", Motor.GoBILDA.RPM_312);
        Motor up_e = new Motor(hardwareMap, "drive_up_e", Motor.GoBILDA.RPM_312);
        Motor up_c = new Motor(hardwareMap, "drive_up_c", Motor.GoBILDA.RPM_312);
        //MecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight)
        //mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        //mecanum = new MecanumDrive(up_c, up_e, in_c, in_e); // fb good, lr reverse, right counter clockwise
        mecanumDrive = new MecanumDrive(in_c, in_e, up_c, up_e); // left correct, turn reverse? right counter clockwise
        //mecanum = new MecanumDrive(up_e, up_c, in_e, in_c); // fb rev, lr rev, right counter clockwise
        //mecanum = new MecanumDrive(in_e, in_c, up_e, up_c); // fb rev, lr cor, right counter clockwise

        motorIntake = hardwareMap.get(DcMotorEx.class, "intake");
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorLift = hardwareMap.get(DcMotorEx.class, "lift_c");
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
        motorLift.setPower(1);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("State", state.name());
        buttonStatePrevious.readValue();
        buttonStateNext.readValue();

        liftTriggerLeft = liftOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        liftTriggerRight = liftOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // driver controls
        double lx = driverOp.getLeftX();
        double ly = driverOp.getLeftY();
        double rx = driverOp.getRightX();
        double degrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        if (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.4 && driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .4) {
            mecanumDrive.driveRobotCentric( -lx, -ly, -rx,true);
        } else {
            mecanumDrive.driveRobotCentric(lx, ly, rx, true);
        }
        telemetry.addData("degrees", "%.2f Deg. (Heading)", degrees);

        // read controls

        switch (state) {
            case INTAKE:
                intake();
                break;
            case TRANSFER_START:
                transferStart();
                break;
            case TRANSFER_PINCH:
                transferPinch();
                break;
            case TRANSFER_FLIP:
                transferFlip();
                break;
            case PLACE_PIXEL:
                placePixel();
                break;
            case HANG:
                hang();
                break;
            case INTAKE_PREP:
                prepIntake();
                break;
        }

        telemetry.addData("Lift Target", motorLift.getTargetPosition());
        telemetry.addData("Lift CurPos", motorLift.getCurrentPosition());


        servoPixelFlip.setPosition(pixelFlipPosition);
        servoPixelRotate.setPosition(pixelRotatePosition);
    }


    public void intake() {
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_START;
            stateRuntime.reset();
            motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
        }
        if (liftTriggerRight > 0.1) {
            // spin intake in
            motorIntake.setPower(STEMperFiConstants.INTAKE_SPEED * liftTriggerRight);

        } else if (liftTriggerLeft > 0.1) {
            // eject pixels
            motorIntake.setPower(-liftTriggerLeft);
            telemetry.addData("Intake", -liftTriggerLeft);

        } else {
            // keep plate flat so pixels don't slide down
            motorIntake.setPower(0);

        }
    }


    public void transferStart() {
        if (buttonStatePrevious.wasJustPressed()) {
            state = STEMperFiConstants.STATE.INTAKE;
            stateRuntime.reset();
        }
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_PINCH;
            stateRuntime.reset();
        }
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        // have to setTargetPosition here if using isBusy
        //motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
        if (!motorLift.isBusy()) {
            pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
            pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_TRANSFER;

            manualLift();
        }
    }

    public void transferPinch() {
        if (buttonStatePrevious.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_START;
            stateRuntime.reset();
        }
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_FLIP;
            stateRuntime.reset();
        }

        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_PINCH);
        if (!motorLift.isBusy()) {
            servoPixelRight.setPosition(STEMperFiConstants.PINCH_CLOSED);
            servoPixelLeft.setPosition(STEMperFiConstants.PINCH_CLOSED);

        }
    }

    public void transferFlip() {
        if (buttonStatePrevious.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_PINCH;
            stateRuntime.reset();
        }
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.PLACE_PIXEL;
            stateRuntime.reset();
        }
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
        if (!motorLift.isBusy()) {
            pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_BACKDROP;
        }
    };

    public void placePixel() {
        if (buttonStatePrevious.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_FLIP;
            stateRuntime.reset();
        }
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.HANG;
            stateRuntime.reset();
        }
        buttonLiftTop.readValue();
        buttonLiftLeft.readValue();
        buttonLiftRight.readValue();
        buttonLiftDown.readValue();

        if (buttonLiftDown.wasJustPressed()) {
            pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
        } else if (buttonLiftRight.wasJustPressed()) {
            pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_RIGHT;
        } else if (buttonLiftLeft.wasJustPressed()) {
            pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_LEFT;
        } else if (buttonLiftTop.wasJustPressed()) {
            pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
        }
        manualLift();

        if (liftOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        }
        if (liftOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        }

    };

    public void hang() {
        if (buttonStatePrevious.wasJustPressed()) {
            state = STEMperFiConstants.STATE.PLACE_PIXEL;
            stateRuntime.reset();
        }
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.INTAKE_PREP;
            stateRuntime.reset();
        }

        pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
        if (stateRuntime.milliseconds() > 500) {
            pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_HANG;
            manualLift();
        }
    }

    public void prepIntake() {

        pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        // have to setTargetPosition here if using isBusy
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
        if (!motorLift.isBusy()) {
            double stateDuration = stateRuntime.milliseconds();
            if (stateDuration > 1000) {
                pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
                motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
                state = STEMperFiConstants.STATE.INTAKE;
                stateRuntime.reset();
            } else if (stateDuration > 500) {
                // give robot half a second to rotate to vertical
                pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;
            }
        }
    };

    void manualLift() {
        int targetPos = motorLift.getTargetPosition();
        targetPos += Math.round(liftOp.getLeftY() * 20);
        targetPos = MathUtils.clamp(targetPos, STEMperFiConstants.LIFT_TARGET_INTAKE, STEMperFiConstants.LIFT_TARGET_HANG);
        motorLift.setTargetPosition(targetPos);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
