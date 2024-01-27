package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
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

    GamepadEx driverOp, liftOp;

    ButtonReader buttonGreen, buttonRed, buttonBlue;

    ToggleButtonReader buttonPink;

    boolean isBlue = true;
    boolean isBackdrop = true;
    DcMotorEx motorIntake, motorLift;
    Servo servoPlate, servoArm, servoPixelRotate, servoPixelFlip, servoPixelLeft, servoPixelRight;
    double pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
    double pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;
    MecanumDrive mecanumDrive;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void initRobot() {


        motorLift = hardwareMap.get(DcMotorEx.class, "lift_c");
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
        motorLift.setPower(.4);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
/*
        frontLeft = hardwareMap.get(DcMotorEx.class, "drive_up_e");
        frontRight = hardwareMap.get(DcMotorEx.class, "drive_up_c");;
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotorEx.class, "drive_in_e");
        backRight = hardwareMap.get(DcMotorEx.class, "drive_in_c");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{ frontLeft, backLeft, frontRight, backRight};
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


        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();;

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

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
/*
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx[] driveMotors;
*/
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
        //stopAndReset();

        telemetry.addData("Waiting for Start!", "!!!!");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            mecanumDrive.driveRobotCentric(0, .2, 0, false);
            do {
                odometry.updatePose();
                telemetry.addData("x", odometry.getPose().getX());
                telemetry.addData("y", odometry.getPose().getY());
                telemetry.update();
            } while (odometry.getPose().getX() > -31);
            mecanumDrive.driveRobotCentric(0, 0, 0, false);
            sleep(2_000);
            mecanumDrive.driveRobotCentric(0, -0.2, 0, false);
            do {
                odometry.updatePose();
                telemetry.addData("x", odometry.getPose().getX());
                telemetry.addData("y", odometry.getPose().getY());
                telemetry.update();
            } while (odometry.getPose().getX() < -21);
            mecanumDrive.driveRobotCentric(0, 0, 0, false);
            motorLift.setTargetPosition(0);
            //moveForwardTicks(2_000, .2);
            sleep(2_000);
            //moveForwardTicks(-1_000, .2);
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
    }

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

    public void stopAndReset() {
        for (DcMotorEx motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

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
