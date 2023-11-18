package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;

/**
 * Created by Michaela on 1/3/2018.
 * Demonstrates the use of a HeadingableMecanumDrivetrain to autonomously rotate a robot to specific headings and hold those headings.
 * Tested and found fully functional by Gabriel on 2018-8-4.
 */


@Autonomous(name = "Auto", group = "Meet 1")
public class Meet1Auto extends LinearOpMode {
    GamepadEx driverOp, liftOp;

    ButtonReader buttonGreen, buttonRed, buttonBlue;

    ToggleButtonReader buttonPink;

    boolean isBlue = true;
    boolean isBackdrop = true;

    Servo servoPlate, servoArm, servoPixelRotate, servoPixelFlip, servoPixelLeft, servoPixelRight;
    double pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_HORIZONTAL;
    double pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;
    double platePosition = STEMperFiConstants.PLATE_FLAT;
    double armPosition = STEMperFiConstants.PLATE_ARM_INTAKE;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void initRobot() {
        servoPlate = hardwareMap.get(Servo.class, "plate");
        servoPlate.setPosition(platePosition);
        servoArm = hardwareMap.get(Servo.class, "arm");
        servoArm.setPosition(armPosition);
        servoPixelRotate = hardwareMap.get(Servo.class, "rotate");
        servoPixelRotate.setPosition(pixelRotatePosition);
        servoPixelFlip = hardwareMap.get(Servo.class, "flip");
        servoPixelFlip.setPosition(pixelFlipPosition);


        // the extended gamepad object

        liftOp = new GamepadEx(gamepad2);

        buttonGreen = new ButtonReader(liftOp, GamepadKeys.Button.Y);
        buttonRed = new ButtonReader(liftOp, GamepadKeys.Button.B);
        buttonBlue = new ButtonReader(liftOp, GamepadKeys.Button.A);
        buttonPink = new ToggleButtonReader(liftOp, GamepadKeys.Button.X);

//        Motor in_e = new Motor(hardwareMap, "drive_in_e", Motor.GoBILDA.RPM_312);
//        Motor in_c = new Motor(hardwareMap, "drive_in_c", Motor.GoBILDA.RPM_312);
//        Motor up_e = new Motor(hardwareMap, "drive_up_e", Motor.GoBILDA.RPM_312);
//        Motor up_c = new Motor(hardwareMap, "drive_up_c", Motor.GoBILDA.RPM_312);

        frontLeft = hardwareMap.get(DcMotorEx.class, "drive_up_e");
        frontRight = hardwareMap.get(DcMotorEx.class, "drive_up_c");;
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotorEx.class, "drive_in_e");
        backRight = hardwareMap.get(DcMotorEx.class, "drive_in_c");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{ frontLeft, backLeft, frontRight, backRight};
/*
        do {
            buttonGreen.readValue();
            buttonBlue.readValue();
            buttonRed.readValue();
            buttonPink.readValue();
            isBackdrop = buttonPink.getState();
            if (buttonRed.wasJustPressed()) {
                isBlue = false;
            }
            if (buttonBlue.wasJustPressed()) {
                isBlue = true;
            }
            telemetry.addData("color: ", isBlue ? "Blue" : "Red");
            telemetry.addData("Backdrop: ", isBackdrop);
            telemetry.addData("Press Green To Arm", "!");
            telemetry.update();
        } while (!buttonGreen.isDown());
*/

    }

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx[] driveMotors;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        stopAndReset();

        telemetry.addData("Waiting for Start!", "!!!!");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            moveForwardTicks(1_550, .2);
            sleep(2_000);
            moveForwardTicks(-1_000, .2);
        }
    }
/*
    public void turnRight(int degrees, double power) {
        stopAndReset();
        frontRight.set(-power);
        backRight.set(-power);
        frontLeft.set(power);
        backLeft.set(power);
        long ticks = STEMperFiConstants.TICKS_PER_DEGREE * degrees;
        long cp = 0;
        do {
            long rcp = frontRight.getCurrentPosition();
            long lcp = frontLeft.getCurrentPosition();
            long mcp = backLeft.getCurrentPosition();
            cp = mcp;
            telemetry.addData("tr rcp: ", rcp);
            telemetry.addData("tr lcp: ", lcp);
            telemetry.addData("tr mcp: ", mcp);
            telemetry.addData("tr target: ", ticks);
            telemetry.update();
        } while (cp < ticks);
        stopAndReset();
    }

    public void turnLeft(int degrees, double power) {
        stopAndReset();
        frontRight.set(power);
        backRight.set(power);
        frontLeft.set(-power);
        backLeft.set(-power);
        long ticks = - STEMperFiConstants.TICKS_PER_DEGREE * degrees;

        long cp = 0;
        do {
            long rcp =frontRight.getCurrentPosition();
            long lcp =frontLeft.getCurrentPosition();
            long mcp = backLeft.getCurrentPosition();
            cp = mcp;
            telemetry.addData("tl rcp: ", rcp);
            telemetry.addData("tl lcp: ", lcp);
            telemetry.addData("tl mcp: ", mcp);
            telemetry.addData("tl target: ", ticks);
            telemetry.update();
        } while (cp > ticks);
        stopAndReset();
    }
*/
    public void moveForwardTicks(int ticks, double power) {
        stopAndReset();
        for (DcMotorEx motor : driveMotors) {
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        do {
            telemetry.addData("fw_mm   fl.cur: ", frontLeft.getCurrentPosition());
            telemetry.addData("fw_mm   fl.tar: ", ticks);
            telemetry.update();
        } while (frontLeft.isBusy());
        stopAndReset();
    }
/*
    public void moveForwardMM(long mm, double power) {
        int ticks = (int) (mm * STEMperFiConstants.TICKS_PER_MM);
        stopAndReset();
        for (Motor motor : driveMotors) {
            motor.setTargetPosition(ticks);
            motor.set(power);
        }
        do {
            telemetry.addData("fw_mm   fl.cur: ", frontLeft.getCurrentPosition());
            telemetry.addData("fw_mm   fl.tar: ", ticks);
            telemetry.update();
        } while (!frontLeft.atTargetPosition());
        stopAndReset();
    }

    public void moveBackwardsMM(long mm, double power) {
        moveForwardMM(-mm, power);
    }
*/
    public void stopAndReset() {
        for (DcMotorEx motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
/*
    // Move the robot forwards
    public void moveForwardCM(double cm, double power) {
        int ticks = (int) (cm * STEMperFiConstants.TICKS_PER_CM);
        stopAndReset();
        for (Motor motor : driveMotors) {
            motor.setTargetPosition(ticks);
            motor.set(power);
        }
        do {
            telemetry.addData("fw_cm   fl.cur: ", frontLeft.getCurrentPosition());
            telemetry.addData("fw_cm   fl.tar: ", ticks);
            telemetry.update();
        } while (!frontLeft.atTargetPosition());
        stopAndReset();
    }

    // Move the robot backwards
    public void moveBackwardsCM(double cm, double power) {
        // backwards is just negative forwards
        moveForwardCM(-cm, power);
    }

    // strafe robot to the left
    public void slideLeftTime(int miliseconds, double power) {
        stopAndReset();

        frontRight.set(power);
        backLeft.set(power);
        frontLeft.set(-power);
        backRight.set(-power);

        sleep(miliseconds);

        stopAndReset();
    }

    // strafe robot to the right
    public void slideRightTime(int miliseconds, double power) {
        stopAndReset();

        frontRight.set(-power);
        backLeft.set(-power);
        frontLeft.set(power);
        backRight.set(power);

        sleep(miliseconds);

        stopAndReset();
    }

    public void pullUpEncoders() {
        //     encoderServoLeft.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_LEFT);
        //    encoderServoRight.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_RIGHT);
        //  encoderServoCenter.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_CENTER);
    }

    // rotate the robot
    public void turn(int ticks, double power) {
        for (Motor motor : driveMotors) {
            motor.resetEncoder();
        }

        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);

        for (Motor motor : driveMotors) {
            motor.setRunMode(Motor.RunMode.PositionControl);
            motor.set(power);
        }

        while (!backLeft.atTargetPosition() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", backLeft.getCurrentPosition());
            telemetry.update();
        }
    }
*/
}
