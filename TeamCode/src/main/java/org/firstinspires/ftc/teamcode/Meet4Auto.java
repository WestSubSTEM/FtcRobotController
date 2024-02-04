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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public static final double TRACKWIDTH = 15.375;
    public static final double CENTER_WHEEL_OFFSET = -5.375;
    public static final double WHEEL_DIAMETER = 1.89;
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

                break;

            default:
                telemetry.addLine("Unknown state");
        }

    }

    public void moveCode() {

        mecanumDrive.driveRobotCentric(0, 0, .3, false);
        sleep(30000);
        //Robot Drives Forward To Place The Pixel
        mecanumDrive.driveRobotCentric(0, .2, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() > -31);

        //Robot Stops And Waits
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(500);

        //Robot Turns To Make Sure The Pixel Is In Place
        mecanumDrive.driveRobotCentric(0, 0, .3, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() < 0);

        //Robot Stops And Lowers The Arm
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        motorLift.setTargetPosition(0);

        //Robot Reverses

        mecanumDrive.driveRobotCentric(0, -0.2, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() < -4);

        //Robot Stops And Waits
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        motorLift.setTargetPosition(0);
        sleep(2_000);

        mecanumDrive.driveRobotCentric(0, 0, .3, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() < 90);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //moveForwardTicks(-1_000, .2);
        mecanumDrive.driveRobotCentric(0, .4, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() > -64);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(1_000);
        mecanumDrive.driveRobotCentric(0, 0, -.3, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData("Angle", odometry.getPose().getRotation().getDegrees());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() > 0);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        mecanumDrive.driveRobotCentric(0, .4, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() > -24);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        //Turn To Align With Backboard
        mecanumDrive.driveRobotCentric(0, 0, .3, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() > 90);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        score(-84);
        /*//Robot Reverses

        mecanumDrive.driveRobotCentric(0, -0.2, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() < -29);

        //Robot Stops And Waits
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        motorLift.setTargetPosition(0);
        sleep(2_000);*/
        /*  Blue Through
        mecanumDrive.driveRobotCentric(0, 0, .3, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() < 90);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        motorLift.setTargetPosition(0);
        //moveForwardTicks(2_000, .2);
        sleep(2_000);
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_PINCH);
        motorLift.setPower(.4);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //moveForwardTicks(-1_000, .2);
        mecanumDrive.driveRobotCentric(0, .4, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() > -72);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(1_000);*/
        //score();
    }

    public void score(int pos)
    {

        //Robot Gets Aligned With The Backdrop
        mecanumDrive.driveRobotCentric(0, 0, .3, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getRotation().getDegrees() < 90);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);


        //Robot Sets The Arm To Scoring Position
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_FLIP);
        sleep(2000);
        servoPixelFlip.setPosition(STEMperFiConstants.PINCH_FLIP_BACKDROP);
        sleep(2000);
        servoPixelRotate.setPosition(STEMperFiConstants.PINCH_ROTATE_VERTICAL);

        //Robot Moves Forward To Score
        mecanumDrive.driveRobotCentric(0, .3, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() > pos);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(500);

        //Robot Scores
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        sleep(10);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        sleep(1000);

        //Robot Backs Up
        mecanumDrive.driveRobotCentric(0, -.3, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() < -80);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        sleep(1000);
        motorLift.setTargetPosition(0);
        sleep(1000);
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}