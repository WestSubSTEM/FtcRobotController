package org.firstinspires.ftc.teamcode;

import android.util.Size;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;

@Autonomous(name = "Auto2", group = "Meet 2")
public class Meet2Auto extends OpMode {
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

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TeamPropDetector tpdProcessor;

    int state;
    int regionGuess;
    int colorGuess;

    @Override
    public void init() {

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

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();;

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

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
        tpdProcessor = new TeamPropDetector();

        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        VisionPortal.Builder vpBuilder = new VisionPortal.Builder();
        vpBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpBuilder.addProcessor(tpdProcessor);
        vpBuilder.setCameraResolution(new Size(640, 480));
        visionPortal = vpBuilder.build();

    }

    @Override
    public void start() {
        state = 0;
        colorGuess = 0;
        regionGuess = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        switch (state) {

            // Detect the location of the team prop
            case 0:
                if (tpdProcessor.guessed()) {
                    telemetry.addLine("Guessing");
                    this.colorGuess = tpdProcessor.getColorGuess();
                    this.regionGuess = tpdProcessor.getRegionGuess();
                    telemetry.addData("Calls", tpdProcessor.getNumberOfCalls());
                    telemetry.addData("Color", this.colorGuess);
                    telemetry.addData("Region", this.regionGuess);
                    telemetry.addData("Largest Area Found", tpdProcessor.getLargestAreaFound());
                    //tpdProcessor.disable();
                    //state = 1;
                } else {
                    telemetry.addLine("Guessing not ready yet");
                }
                break;

            // Push pre-loaded purple pixel to the proper Spike Mark (20 points)'
            case 1:
                moveToSpikeMark(this.colorGuess, this.regionGuess);
                break;

            // Move to the correct backdrop
            case 2:
                break;

            // Place the pre-loaded yellow pixel on the backdrop (20 points)
            case 3:
                break;

            // Go to the white pixel storage area w/o disturbing the purple pixel
            case 4:
                break;

            // Intake a white pixel
            case 5:
                break;

            // Go to the Backdrop
            case 6:
                break;

            // Place the white pixel on the backdrop (3 or 5 points)
            case 7:
                break;

            default:
                telemetry.addLine("Auto complete");

        }
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void moveToSpikeMark(int colorGuess, int regionGuess) {
        switch (regionGuess) {
            case 1:
                // Move to the left

                // Move forward
                mecanumDrive.driveRobotCentric(0, .2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() > -31);

                // Turn left
                mecanumDrive.driveRobotCentric(0, 0, -.2, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("Rotation", odometry.getPose().getRotation());
                } while (odometry.getPose().getRotation().getDegrees() > -90);

                // Move forward
                mecanumDrive.driveRobotCentric(0, .2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() > -31);

                // Stop
                mecanumDrive.driveRobotCentric(0, 0, 0, false);
                sleep(2_000);

                // Move backward
                mecanumDrive.driveRobotCentric(0, -0.2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() < -21);
                mecanumDrive.driveRobotCentric(0, 0, 0, false);

                // Drop lift
                motorLift.setTargetPosition(0);
                sleep(2_000);
                break;
            case 2:
                // Move to the center
                // Move to the right
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
                sleep(2_000);
                break;
            case 3:
                // Move to the right
                break;
            default:
                telemetry.addLine("Invalid spike mark zone");
        }
    }
}
