package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.processors.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



@Autonomous(name = "Aidan and Adam", group = "Meet 2")
public class Aidan_And_Adam extends LinearOpMode {
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
    int state_Auto;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TeamPropDetector tpdProcessor;

    int state;
    int regionGuess;
    int colorGuess;


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
        //servoPixelRotate.setPosition(pixelRotatePosition);
        servoPixelFlip = hardwareMap.get(Servo.class, "flip");
        //servoPixelFlip.setPosition(pixelFlipPosition);
        servoPixelLeft = hardwareMap.get(Servo.class, "left");
        //servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight = hardwareMap.get(Servo.class, "right");
        //servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
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
        centerOdometer.reset();
        ;

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );


    }

    public void moveAndBackWords(double dist, double speed, double turnSpeed, int delay) {
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

    public void moveBackwordsY(double dist, double speed, int delay) {
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

    public void moveBackwordsX(double dist, double speed, int delay) {
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

    public void score(int pos) {

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

    public void moveDiagnal(double hypotinus, double angle, double speed) {
        moveForwardY(hypotinus * Math.sin(angle * (Math.PI / 180)), speed, 0);
    }

    public void moveDiagnalBackwords(double hypotinus, double angle, double speed) {
        moveBackwordsY(hypotinus * Math.sin(angle * (Math.PI / 180)), speed, 0);
    }

    public void setArmHight(int Hight) {
        motorLift.setTargetPosition(Hight);
    }

    public void center()
    {
        moveForwardX(-26,.3,500);
        moveBackwordsX(-5,-.3,0);
        turn(94,.2,0);
    }
    public void rightFar(
    {
        moveAndTurn(-20,.3,-.1,0);
        sleep(500);
        moveAndBackWords(-2,-.3,.1,0);
        moveForwardX(-4,.2,0);
        turn(94,.2,0);
    }
    public void left()
    {
        moveForwardX(-13,.3,0);
        turn(30,.2,0);
        moveDiagnal(-5.7,30,.3);
        sleep(500);
        moveDiagnalBackwords(11.5,30,-.3);
        sleep(500);
        turn(93,.3,0);
    }
    public void rightClose()
    {
        moveForwardX(-20,.3,0);
        turnRight(-60,-.2,0);
        moveDiagnalBackwords(12,30,.3);
        sleep(500);
        moveDiagnal(-7,30,-.3);
        turn(0,.2,0);
        moveBackwordsX(-6,-.3,500);
        turn(90,.2,0);
    }

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
        /*
            |      |     ________       I have commented all the code so it should be easy to understand
            |      |        |
            |______|        |
            |      |        |
            |      |     ________

         */
        state_Auto = 2;
        boolean badTeamMate = false;
        if (opModeIsActive()) {
            //If the camera reads the team marker is the center location the robot will execute this code
            if(state_Auto == 0)
            {  center();
                //moveForwardY(-19.2,.4,0);
                  moveForwardY(-68,.4,0);


            }
            //If the camera reads the team marker is the right location the robot will execute this code
            else if(state_Auto == 1)
            {
                rightFar();
                //moveForwardY(-19.2,.4,0);
               moveForwardY(-68,.4,0);


            }
            //If the camera reads the team marker is the left location the robot will execute this code
            else
            {
                left();
                //moveForwardY(-19.2,.4,0);
                moveForwardY(-68,.4,0);

            }
        }
    }
}