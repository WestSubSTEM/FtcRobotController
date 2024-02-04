package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "Meet 4 Auto", group = "Meet 4")
public class Meet4Auto extends OpMode {
    public static final double TRACK_WIDTH = 15.375;
    public static final double CENTER_WHEEL_OFFSET = -5.375;
    public static final double WHEEL_DIAMETER = 1.89;
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    GamepadEx driverOp, liftOp;
    ButtonReader buttonGreen, buttonRed, buttonBlue;
    ToggleButtonReader buttonPink;
    boolean isBackdrop = true;
    DcMotorEx motorIntake, motorLift;
    Servo servoPlate, servoArm, servoPixelRotate, servoPixelFlip, servoPixelLeft, servoPixelRight;
    double pixelRotatePosition = STEMperFiConstants.PINCH_ROTATE_VERTICAL;
    double pixelFlipPosition = STEMperFiConstants.PINCH_FLIP_INTAKE;
    MecanumDrive mecanumDrive;

    // Vision

    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    TeamPropDetector tpDetector;

    int state;
    int regionGuess;
    STEMperFiConstants.TeamPropColor colorGuess;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addLine("Initializing motors and drive");

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
        servoPixelLeft = hardwareMap.get(Servo.class, "left");
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight = hardwareMap.get(Servo.class, "right");
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        sleep(3000);
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_CLOSED);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_CLOSED_WALL);
        // the extended gamepad object

        liftOp = new GamepadEx(gamepad2);

        buttonGreen = new ButtonReader(liftOp, GamepadKeys.Button.Y);
        buttonRed = new ButtonReader(liftOp, GamepadKeys.Button.B);
        buttonBlue = new ButtonReader(liftOp, GamepadKeys.Button.A);
        buttonPink = new ToggleButtonReader(liftOp, GamepadKeys.Button.X);

        Motor in_e = new Motor(hardwareMap, "drive_in_e", Motor.GoBILDA.RPM_312);
        Motor in_c = new Motor(hardwareMap, "drive_in_c", Motor.GoBILDA.RPM_312);
        Motor up_e = new Motor(hardwareMap, "drive_up_e", Motor.GoBILDA.RPM_312);
        Motor up_c = new Motor(hardwareMap, "drive_up_c", Motor.GoBILDA.RPM_312);
        mecanumDrive = new MecanumDrive(in_c, in_e, up_c, up_e); // left correct, turn reverse? right counter clockwise

        leftOdometer = in_e.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = in_c.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = up_c.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        leftOdometer.setDirection(Motor.Direction.REVERSE);


        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();
        ;

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        // TeamPropDetector processor
        telemetry.addLine("Initializing TeamPropDetector processor");
        tpDetector = new TeamPropDetector();

        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        VisionPortal.Builder vpBuilder = new VisionPortal.Builder();
        vpBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpBuilder.addProcessor(tpDetector);
        vpBuilder.setCameraResolution(new Size(640, 480));
        visionPortal = vpBuilder.build();

    }

    @Override
    public void start() {
        state = 0;
        colorGuess = STEMperFiConstants.TeamPropColor.UNKNOWN;
        regionGuess = 0;
        tpDetector.reset();
    }

    @Override
    public void loop() {
        //stopAndReset();

        telemetry.addData("State", state);

        switch (state) {

            case 0:
                if (tpDetector.isGuessed()) {

                    this.colorGuess = tpDetector.getColorGuess();
                    this.regionGuess = tpDetector.getRegionGuess();
                    tpDetector.disable();
                    state = 1;
                }
                sleep(200); // Polite pause to allow for interrupts
                break;

            case 1:

                telemetry.addData("Color Guess", this.colorGuess);
                telemetry.addData("Region Guess", this.regionGuess);
                telemetry.addLine("Moving to spike mark");
                moveToSpikeMark(this.regionGuess);
                state = 2;
                break;

            default:
                telemetry.addLine("Unknown state");
        }

    }

    private void moveToSpikeMark(int region) {
        switch (region) {
            case 1:
                left();
                //moveForwardY(-19.2,.4,0);
                moveForwardY(-68, .4, 0);
                break;
            case 2:
                center();
                //moveForwardY(-19.2,.4,0);
                moveForwardY(-68, .4, 0);
                break;
            case 3:
                rightFar();
                //moveForwardY(-19.2,.4,0);
                moveForwardY(-68, .4, 0);
                break;
        }
    }


    public void moveAndBackwards(double dist, double speed, double turnSpeed, int delay) {
        mecanumDrive.driveRobotCentric(0, speed, turnSpeed, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() < dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void moveAndTurn(double dist, double speed, double turnSpeed, int delay) {
        mecanumDrive.driveRobotCentric(0, speed, turnSpeed, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() > dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void moveBackwardsY(double dist, double speed, int delay) {
        mecanumDrive.driveRobotCentric(0, speed, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() < dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void moveForwardY(double dist, double speed, int delay) {
        mecanumDrive.driveRobotCentric(0, speed, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() > dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void moveBackwardsX(double dist, double speed, int delay) {
        mecanumDrive.driveRobotCentric(0, speed, 0, false);
        do {

            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() < dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void moveForwardX(double dist, double speed, int delay) {
        mecanumDrive.driveRobotCentric(0, speed, 0, false);
        do {

            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() > dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void turnRight(int angle, double speed, int delay) {

        mecanumDrive.driveRobotCentric(0, 0, speed, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData(" Angle", odometry.getPose().getRotation().getDegrees());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() > angle);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void turn(int angle, double speed, int delay) {

        mecanumDrive.driveRobotCentric(0, 0, speed, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData(" Angle", odometry.getPose().getRotation().getDegrees());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() < angle);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(delay);
    }

    public void moveDiagonal(double hypotenuse, double angle, double speed) {
        moveForwardY(hypotenuse * Math.sin(angle * (Math.PI / 180)), speed, 0);
    }

    public void moveDiagonalBackwards(double hypotenuse, double angle, double speed) {
        moveBackwardsY(hypotenuse * Math.sin(angle * (Math.PI / 180)), speed, 0);
    }

    public void setArmHeight(int height) {
        motorLift.setTargetPosition(height);
    }

    public void center() {
        moveForwardX(-26, .3, 500);
        moveBackwardsX(-5, -.3, 0);
        turn(94, .2, 0);
    }

    public void rightFar() {
        moveAndTurn(-20, .3, -.1, 0);
        sleep(500);
        moveAndBackwards(-2, -.3, .1, 0);
        moveForwardX(-4, .2, 0);
        turn(94, .2, 0);
    }

    public void left() {
        moveForwardX(-13, .3, 0);
        turn(30, .2, 0);
        moveDiagonal(-5.7, 30, .3);
        sleep(500);
        moveDiagonalBackwards(11.5, 30, -.3);
        sleep(500);
        turn(93, .3, 0);
    }

    public void rightClose() {
        moveForwardX(-20, .3, 0);
        turnRight(-60, -.2, 0);
        moveDiagonalBackwards(12, 30, .3);
        sleep(500);
        moveDiagonal(-7, 30, -.3);
        turn(0, .2, 0);
        moveBackwardsX(-6, -.3, 500);
        turn(90, .2, 0);
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}