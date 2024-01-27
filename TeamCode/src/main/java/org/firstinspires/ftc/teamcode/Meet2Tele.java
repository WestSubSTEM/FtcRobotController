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

import android.graphics.Color;
import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

@TeleOp(name="Tele", group="Meet 2")
public class Meet2Tele extends OpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 15.375;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -5.375;

    public static final double WHEEL_DIAMETER = 1.89;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    private QwiicLEDStrip ledStripFront;

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
    double pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
    double pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;

    STEMperFiConstants.STATE state = STEMperFiConstants.STATE.INTAKE;

    ButtonReader buttonStateNext, buttonStatePrevious;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TeamPropDetector teamPropDetector;
    private WebcamName webCam;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        webCam = hardwareMap.get(WebcamName.class, "Webcam 1");

        ledStripFront = hardwareMap.get(QwiicLEDStrip.class, "led");
        ledStripFront.setBrightness(2);
        ledStripFront.setColor(STEMperFiConstants.COLOR_RED);

        servoPixelRotate = hardwareMap.get(Servo.class, "rotate");
        servoPixelRotate.setPosition(pixelRotatePosition);
        telemetry.addData("rotate", pixelRotatePosition);

        servoPixelFlip = hardwareMap.get(Servo.class, "flip");
        servoPixelFlip.setPosition(pixelFlipPosition);
        telemetry.addData("flip", pixelFlipPosition);

        servoPixelLeft = hardwareMap.get(Servo.class, "left");
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
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

        leftOdometer = in_e.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = in_c.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = up_c.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        leftOdometer.setDirection(Motor.Direction.REVERSE);


        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();;

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );


        motorIntake = hardwareMap.get(DcMotorEx.class, "intake");
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorLift = hardwareMap.get(DcMotorEx.class, "lift_c");
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setPower(1);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // VisionPortal
        // AprilTag processor
        /*
        telemetry.addLine("Initializing april tag processor");
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        */

        // TeamPropDetector processor
        telemetry.addLine("Initializing TeamPropDetector processor");
        teamPropDetector = new TeamPropDetector();
        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        /*
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor,
                tpdProcessor);
         */
        /*
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                teamPropDetector);

        VisionPortal.Builder vpBuilder = new VisionPortal.Builder();
        vpBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpBuilder.addProcessor(teamPropDetector);
        vpBuilder.setCameraResolution(new Size(720, 480));
        visionPortal = vpBuilder.build();

         */
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("left",leftOdometer.getPosition());
        telemetry.addData("right",rightOdometer.getPosition());
        telemetry.addData("center",centerOdometer.getPosition());
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
        // vision portal
        /*
        telemetry.addData("TeamPropDetector Calls", teamPropDetector.getCalls());
        telemetry.addData("TeamPropDetector No. Pixels", teamPropDetector.getNumberOfPixels());
        if (teamPropDetector.foundPixels()) {
            int zone = teamPropDetector.getSpikeMarkZone();
            telemetry.addData("Team Prop", "found in " + zone);
        } else {
            telemetry.addData("Team Prop", "not found");
        }

         */


        telemetry.addData("State", state.name());
        buttonStatePrevious.readValue();
        buttonStateNext.readValue();

        liftTriggerLeft = liftOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        liftTriggerRight = liftOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // driver controls
        double lx = driverOp.getLeftX();
        double ly = driverOp.getLeftY();
        double rx = driverOp.getRightX();
        odometry.updatePose();
        mecanumDrive.driveFieldCentric(lx, ly, rx, odometry.getPose().getRotation().getDegrees(), gamepad1.left_bumper || gamepad1.right_bumper);


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
            case DRIVE_TO_HUMAN:
                driveToHuman();
            break;
        }

        telemetry.addData("Lift Target", motorLift.getTargetPosition());
        telemetry.addData("Lift CurPos", motorLift.getCurrentPosition());

        adjustFlipPosition();
        //servoPixelFlip.setPosition(pixelFlipPosition);
        servoPixelRotate.setPosition(pixelRotatePosition);
        telemetry.addData("flip target", pixelFlipPosition);
        telemetry.addData("flip current", servoPixelFlip.getPosition());
        telemetry.addData("rotate", pixelRotatePosition);

    }

    public void adjustFlipPosition() {
        double currentPosition = servoPixelFlip.getPosition();
        double newPosition = 0;
        if (currentPosition < pixelFlipPosition) {
            newPosition = currentPosition + STEMperFiConstants.FLIP_INCREMENT;
            newPosition = Math.min(newPosition, pixelFlipPosition);
        } else {
            newPosition = currentPosition - STEMperFiConstants.FLIP_INCREMENT;
            newPosition = Math.max(newPosition, pixelFlipPosition);
        }
        servoPixelFlip.setPosition(newPosition);
    }

    public boolean flipIsBusy() {
        return servoPixelFlip.getPosition() != pixelFlipPosition;
    }


    public void intake() {
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.TRANSFER_START;
            ledStripFront.turnAllOff();
            stateRuntime.reset();
            motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_PINCH);
        }
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        if (liftTriggerRight > 0.1 && !motorLift.isBusy()) {
            // eject pixels
            motorIntake.setPower(-STEMperFiConstants.INTAKE_SPEED * liftTriggerRight);
        } else if (liftTriggerLeft > 0.1 && !motorLift.isBusy()) {
            // spin intake in
            motorIntake.setPower(STEMperFiConstants.INTAKE_SPEED * liftTriggerLeft);
        } else {
            // keep plate flat so pixels don't slide down
            motorIntake.setPower(0);
        }
    }


    public void transferStart() {
        // have to setTargetPosition here if using isBusy
        //motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
        if (!motorLift.isBusy()) {
            servoPixelLeft.setPosition(STEMperFiConstants.PINCH_CLOSED);
            servoPixelRight.setPosition(STEMperFiConstants.PINCH_CLOSED_WALL);
            state = STEMperFiConstants.STATE.TRANSFER_PINCH;
            stateRuntime.reset();
        }
    }

    public void transferPinch() {
        if (stateRuntime.milliseconds() > 750) {
            motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
            motorLift.setPower(0.3);
            if (!motorLift.isBusy()) {
                pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_BACKDROP;
                if (!flipIsBusy()) {
                    state = STEMperFiConstants.STATE.TRANSFER_FLIP;
                    stateRuntime.reset();
                }
            }
        }
    }

    public void transferFlip() {
        motorLift.setPower(.8);
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_PINCH);
        ledStripFront.setBrightness(2);
        ledStripFront.setColor(STEMperFiConstants.COLOR_GREEN);
        pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
        state = STEMperFiConstants.STATE.PLACE_PIXEL;
    }

    public void placePixel() {
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.HANG;
            ledStripFront.setBrightness(2);
            ledStripFront.setColor(STEMperFiConstants.COLOR_PURPLE);
            pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_HANG;
            pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
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
            servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        }
        if (liftOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        }

        if (liftTriggerRight > 0.1 && !motorLift.isBusy()) {
            // eject pixels
            motorIntake.setPower(-STEMperFiConstants.INTAKE_SPEED * liftTriggerRight);
        } else if (liftTriggerLeft > 0.1 && !motorLift.isBusy()) {
            // spin intake in
            motorIntake.setPower(STEMperFiConstants.INTAKE_SPEED * liftTriggerLeft);
        } else {
            // keep plate flat so pixels don't slide down
            motorIntake.setPower(0);
        }

    };

    public void hang() {
        motorLift.setPower(1);
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.INTAKE_PREP;
            ledStripFront.turnAllOff();
            stateRuntime.reset();
        }
        pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
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
            if (stateDuration > 300 && !flipIsBusy()) {
                motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_PINCH);
                state = STEMperFiConstants.STATE.DRIVE_TO_HUMAN;
                stateRuntime.reset();
                ledStripFront.setBrightness(2);
                ledStripFront.setColor(STEMperFiConstants.COLOR_ORANGE);
            } else if (stateDuration > 250) {
                // give robot half a second to rotate to vertical
                pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;
            }
        } else {
            stateRuntime.reset();
        }
    };

    public void driveToHuman() {
        if (buttonStateNext.wasJustPressed()) {
            state = STEMperFiConstants.STATE.INTAKE;
            ledStripFront.setBrightness(2);
            ledStripFront.setColor(STEMperFiConstants.COLOR_RED);
            motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
            motorLift.setPower(.2);
            stateRuntime.reset();
        }
    }

    void manualLift() {
        int targetPos = motorLift.getTargetPosition();
        targetPos += Math.round(liftOp.getLeftY() * 20);
        targetPos = MathUtils.clamp(targetPos, STEMperFiConstants.LIFT_TARGET_INTAKE, STEMperFiConstants.LIFT_TARGET_MAX);
        motorLift.setTargetPosition(targetPos);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
