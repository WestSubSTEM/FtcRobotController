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

    // If far is true, then robot starts further from the backstage
    boolean far = true;
    boolean parkLeft = true;

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
       // motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
       // motorLift.setPower(.4);
        //motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoPixelRotate = hardwareMap.get(Servo.class, "rotate");
        servoPixelRotate.setPosition(pixelRotatePosition);
        servoPixelFlip = hardwareMap.get(Servo.class, "flip");
        servoPixelFlip.setPosition(pixelFlipPosition);
        servoPixelLeft = hardwareMap.get(Servo.class, "left");
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        servoPixelRight = hardwareMap.get(Servo.class, "right");
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        sleep(1000);
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_CLOSED);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_CLOSED_WALL);
        // the extended gamepad object
        motorLift.setTargetPosition(STEMperFiConstants.LIFT_TARGET_INTAKE);
        motorLift.setPower(.4);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
       /* ButtonReader buttonLiftTop, buttonLiftLeft, buttonLiftRight, buttonLiftDown, buttonDroneLaunch;
        buttonLiftRight = new ButtonReader(liftOp, GamepadKeys.Button.B);
        buttonLiftDown = new ButtonReader(liftOp, GamepadKeys.Button.A);
        ButtonReader buttonStateNext;
        buttonStateNext = new ButtonReader(liftOp, GamepadKeys.Button.DPAD_UP);
*/
        colorGuess = STEMperFiConstants.TeamPropColor.UNKNOWN;
        regionGuess = 0;
        tpDetector.reset();

    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            this.far = false;
        }
        if (gamepad1.triangle) {
            this.far = true;
        }
        telemetry.addLine("Guessing");
        STEMperFiConstants.TeamPropColor tempColorGuess = tpDetector.getColorGuess();
        int tempRegionGuess =  tpDetector.getRegionGuess();
        if (tempColorGuess != STEMperFiConstants.TeamPropColor.UNKNOWN) {
            this.colorGuess = tempColorGuess;
        }
        if (tempRegionGuess != 0) {
            this.regionGuess = tempRegionGuess;
        }
        telemetry.addData("Far", this.far);
        telemetry.addData("Color Guess", this.colorGuess);
        telemetry.addData("Region Guess", this.regionGuess);
        telemetry.update();
    }


    @Override
    public void start() {
        // Set this to 0 to guess at the start or 1 to bypass guessing
        state = 1;
    }

    @Override
    public void loop() {
        //stopAndReset();

        telemetry.addData("State", state);

        switch (state) {

            case 0:
                if (tpDetector.isGuessed()) {
                    telemetry.addLine("Color and region has been guess");

                    this.colorGuess = tpDetector.getColorGuess();
                    this.regionGuess = tpDetector.getRegionGuess();
                    state = 1;
                } else {
                    telemetry.addLine("Guessing color and region");
                }

                sleep(200); // Polite pause to allow for interrupts
                break;

            case 1:
                tpDetector.disable();

                telemetry.addLine("Moving to spike mark then backstage");
                telemetry.addData("Color Guess", this.colorGuess);
                telemetry.addData("Region Guess", this.regionGuess);
                // Move to the spike mark and back up to base position
                moveToSpikeMark();
                state = 2;
                break;

            case 2:

                telemetry.addLine("Placing pixel on score board");
                telemetry.addData("Color Guess", this.colorGuess);
                telemetry.addData("Region Guess", this.regionGuess);
                // TODO Score
                score();
                state = 3;
                break;

            case 3:

                telemetry.addLine("Park");
                telemetry.addData("Color Guess", this.colorGuess);
                telemetry.addData("Region Guess", this.regionGuess);
                // TODO Score
                park();


            default:
                telemetry.addLine("Unknown state");
        }

    }

    /*
    Need to move to the spike mark based on 3 variables:
    - this.colorGuess for the color of the team prop
    - this.regionGuess for the location of the spike mark (1 is to the left of the robot)
    - this.far for whether the robot is starting in further from the backstage (vs. near)

    After moving to the spike mark, the robot should back up a bit to release the pixel
    and then move to the starting spot of the backstage to prepare for the score function.

    The starting spot of the backstage should be positioned in line with the back drop so
    that the pixel can be placed in the first AprilTag slot.
     */
    private void moveToSpikeMark() {

        switch (this.colorGuess) {
            case BLUE:
                switch (this.regionGuess) {
                    case 1:
                        if(far) {
                            moveLeftBlueSpikeMark();
                            moveForwardY(-79, .4, 500);


                        }
                        else
                        {
                            moveLeftBlueSpikeMark();
                            moveForwardY(-30,.4,0);
                        }
                        break;
                    case 2:
                        if(far)
                        {moveCenterBlueSpikeMark();
                        moveForwardY(-78.5, .4, 500);}
                        else
                        {
                            moveCenterBlueSpikeMark();
                            moveForwardY(-30,.4,0);
                        }
                        break;
                    case 3:
                        if (this.far) {
                            moveRightBlueFarSpikeMark();
                        } else {
                            moveRightBlueNearSpikeMark();
                        }
                        break;
                }
                break;

            case RED:
                switch (this.regionGuess) {
                    case 1:
                        if (this.far) {
                            moveLeftRedFarSpikeMark();
                        } else {
                            moveLeftRedNearSpikeMark();
                        }
                        break;
                    case 2:
                            moveCenterRedSpikeMark();
                        break;
                    case 3:
                        moveRightRedSpikeMark();

                        break;
                }
                break;
        }

    }

    /*------------------------------------
    FUNCTIONS FOR MOVING TO THE SPIKE MARK
    ------------------------------------*/

    private void moveLeftBlueSpikeMark() {
        moveForwardX(-13, .3, 0);
        turn(30, .2, 0);
        moveDiagonal(-5.7, 30, .3);
        sleep(500);
        moveDiagonalBackwards(13, 30, -.3);
        sleep(500);
        turn(90, .3, 0);
    }
    private void moveCenterBlueSpikeMark() {
        moveForwardX(-29, .3, 500);
        moveBackwardsX(-5, -.3, 0);
        turn(91.5, .2, 0);

    }

    private void moveRightBlueFarSpikeMark() {
        moveAndTurn(-21.5, .3, -.1, 0);
        sleep(500);
        moveAndBackwards(-2, -.3, .1, 0);
        moveForwardX(-4, .2, 0);
        turn(91, .2, 0);
        moveForwardY(-78, .4, 500);
    }

    private void moveRightBlueNearSpikeMark() {
        moveForwardX(-20,.3,0);
        turnRight(-60,-.2,0);
        moveDiagonalBackwards(17,30,.3);
        sleep(500);
        moveDiagonal(-14,30,-.3);
        turn(0,.2,0);
        moveBackwardsX(-6,-.3,500);
        turn(90,.2,0);
        moveForwardY(-31, .4, 500);
    }

    private void moveLeftRedFarSpikeMark() {

        moveForwardX(-13, .3, 0);
        turn(30, .2, 0);
        moveDiagonal(-8, 30, .3);
        sleep(500);
        moveDiagonalBackwards(15, 30, -.3);
        sleep(500);
        turnRight(-90, -.3, 0);

    }

    private void moveLeftRedNearSpikeMark() {
        // TODO
        moveForwardX(-13, .3, 0);
        turn(30, .2, 0);
        moveDiagonal(-5.7, 30, .3);
        sleep(500);
        moveDiagonalBackwards(13, 30, -.3);
        sleep(500);
        turnRight(-90, -.3, 0);


    }

    private void moveCenterRedSpikeMark() {
        // TODO
        moveForwardX(-29, .3, 500);
        moveBackwardsX(-5, -.3, 0);
        turnRight(-90, -.3, 0);
    }

    private void moveRightRedSpikeMark() {
        // TODO
        moveForwardX(-20,.3,0);
        turnRight(-60,-.2,0);
        moveDiagonalBackwards(17,30,.3);
        sleep(500);
        moveDiagonal(-14,30,-.3);
        turn(0,.2,0);
        moveBackwardsX(-6,-.3,500);
        turn(-90,-.2,0);


    }


    /*-------------------------------------
    FUNCTIONS FOR SCORING ON THE BACK STAGE
    -------------------------------------*/
    private void score() {

        // Move to the correct location
        switch (this.colorGuess) {
            case BLUE:
                switch (this.regionGuess) {
                    case 1:
                        moveLeftBlueScore();
                        break;
                    case 2:
                        moveCenterBlueScore();
                        break;
                    case 3:
                        moveRightBlueScore();
                        break;
                }
                break;
            case RED:
                switch (this.regionGuess) {
                    case 1:
                        moveLeftRedScore();
                        break;
                    case 2:
                        moveCenterRedScore();
                        break;
                    case 3:
                        moveRightRedScore();
                        break;
                }
                break;
        }

        // Raise the arm and place the pixel
        // TODO


    }

    private void moveLeftBlueScore() {
        // TODO
        if(far)
        {strafe(-15,.6);
        Score(-89,0.2);}
        else {
            strafe(-15,.6);
            Score(-40,0.2);
        }
    }

    private void moveCenterBlueScore() {
        if(far)
        {
        strafe(-20.5,.6);
        Score(-89,0.2);}
        else
        {
            strafe(-20.5,.6);
            Score(-40,0.2);
        }
    }

    private void moveRightBlueScore() {
        if(far)
        {strafe(-29,.6);
        Score(-89.6,0.2);}
         else
        {
            strafe(-29,.6);
            Score(-41,0.2);
        }
    }

    private void moveLeftRedScore() {
        // TODO
    }

    private void moveCenterRedScore() {
        // TODO
    }

    private void moveRightRedScore() {
        // TODO
    }

    /*------------------------------------
    FUNCTIONS FOR PARKING IN THE BACK AREA
    ------------------------------------*/

    /*
    Parking is the same for red and blue sides. It will vary based on the region and whether
    we are choosing to park to the right or left of the back drop.
     */
    private void park() {

        switch (this.regionGuess) {
            case 1:
                if (this.parkLeft) {
                    // Strafe to the left a little
                    turnRight(0,-.2,0);
                    mecanumDrive.driveRobotCentric(0, -.3, 0, false);
                    sleep(1500);
                    mecanumDrive.driveRobotCentric(0, 0, 0, false);
                } else {
                    // Strafe to the right a lot
                    // TODO
                }
                break;
            case 2:
                if (this.parkLeft) {
                    // Strafe to the left somewhat
                    turnRight(0,-.2,0);
                    mecanumDrive.driveRobotCentric(0, -.3, 0, false);
                    sleep(1499);
                    mecanumDrive.driveRobotCentric(0, 0, 0, false);
                } else {
                    // Strafe to the right somewhat
                    // TODO
                }
                break;
            case 3:
                if (this.parkLeft) {
                    // Strafe to the left a lot
                    turnRight(0,-.3,0);
                    mecanumDrive.driveRobotCentric(0, -.5, 0, false);
                    sleep(1200);
                    mecanumDrive.driveRobotCentric(0, 0, 0, false);
                    stop();
                } else {
                    // Strafe to the right a little
                    // TODO
                }
                break;

        }

    }

    /*--------------
    HELPER FUNCTIONS
    --------------*/
    private void Score(double dist, double speed)
    {

        int yval = 0;
        //Robot Sets The Arm To Scoring Position
        motorLift.setTargetPosition(STEMperFiConstants.AUTO_SCORE);
        sleep(1000);
        servoPixelFlip.setPosition(STEMperFiConstants.PINCH_FLIP_BACKDROP);
        sleep(1000);
        servoPixelRotate.setPosition(STEMperFiConstants.PINCH_ROTATE_VERTICAL);
        sleep(1000);
        turnRight(90,-.2,0);
        //Robot Moves Forward To Score
        moveForwardY(dist,speed,0);

        //Robot Scores
        sleep(1000);
        servoPixelLeft.setPosition(STEMperFiConstants.PINCH_OPEN);
        sleep(10);
        servoPixelRight.setPosition(STEMperFiConstants.PINCH_OPEN);
        sleep(1000);
        if(far)
        {
            yval = -80;
        }
        else
            yval = -36;
        //Robot Backs Up
        mecanumDrive.driveRobotCentric(0, -.3, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getY() < yval);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
        servoPixelFlip.setPosition(STEMperFiConstants.PINCH_FLIP_INTAKE);
        sleep(1000);
        motorLift.setTargetPosition(0);
        sleep(1000);
    }
    private void strafe(double dist, double speed)
    {
        mecanumDrive.driveRobotCentric(speed, 0, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() > dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
    }
    private void strafeLeft(double dist, double speed)
    {
        mecanumDrive.driveRobotCentric(speed, 0, 0, false);
        do {
            odometry.updatePose();
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.update();
        } while (odometry.getPose().getX() < dist);
        mecanumDrive.driveRobotCentric(0, 0, 0, false);
    }
    private void moveAndBackwards(double dist, double speed, double turnSpeed, int delay) {
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

    private void moveAndTurn(double dist, double speed, double turnSpeed, int delay) {
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

    private void moveBackwardsY(double dist, double speed, int delay) {
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

    private void moveForwardY(double dist, double speed, int delay) {
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

    private void moveBackwardsX(double dist, double speed, int delay) {
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

    private void moveForwardX(double dist, double speed, int delay) {
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

    private void turnRight(int angle, double speed, int delay) {

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

    private void turn(double angle, double speed, int delay) {

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

    private void moveDiagonal(double hypotenuse, double angle, double speed) {
        moveForwardY(hypotenuse * Math.sin(angle * (Math.PI / 180)), speed, 0);
    }

    private void moveDiagonalBackwards(double hypotenuse, double angle, double speed) {
        moveBackwardsY(hypotenuse * Math.sin(angle * (Math.PI / 180)), speed, 0);
    }

    private void setArmHeight(int height) {
        motorLift.setTargetPosition(height);
    }


    private void rightClose() {
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