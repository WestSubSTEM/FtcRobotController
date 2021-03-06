package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;


@Autonomous(name = "Auto", group = "Meet3")
public class AutoMeet3 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =  "AXRxi6H/////AAAAGXFBcOuXBkpfsXQl3RrsaWMs3rPkMqV94uKhxmHX5LE95IW4PqzCg3G44Uqx8hnsDvRnPQrbus1zvbgc+3sPBt4w08IbyebgwgnFN9221SFutmZ76ox5ctJ6+HhTKIyfyYJjSWUaxADTzTy5w8BNnu9KOk6GOiafGNqbDzFffECDcnfSkxQBSlvuTtioONy5dKrhUj6nFuIXIXFO9kb6vqhqjzS6ViKUcSbkYmQ8Pjrqb5W4cUd+wyeGMDqFQkEUlWdm/z/J+p774VeP9NquwDPUVfR4GLUEQsA8/EG0B8IoVG1VCeHZOJcpIiapQOPQ9eMpVaBr+Qj6E0kaEUR5vZ9QFXYDpk+1fpyB1RGGSmAm";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;
    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

    public DcMotor angleMotor;
    public int anglePosition = 0;
    public DcMotor wobbleMotor;
    public int wobbleArmPosition = 0;
    //public DcMotor flywheelMotor;
    public DcMotorEx flywheelMotorEx;
    public DcMotor intakeMotor;

    public Servo triggerServo;
    public double triggerServoPosition = StemperFiConstants.TRIGGER_SERVO_LOAD;

    public Servo wobbleServo;
    public double wobbleServoPosition = StemperFiConstants.WOBBLE_SERVO_OPEN;

    public String targetZone = StemperFiConstants.TARGET_ZONE_A;

    public int targetZoneRight = 30;
    public int targetZoneForward = 40;
    public int targetZonePark = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        while (!opModeIsActive() && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    targetZone = StemperFiConstants.TARGET_ZONE_A;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == StemperFiConstants.TARGET_ZONE_B){
                            targetZone = StemperFiConstants.TARGET_ZONE_B;
                        } else if (recognition.getLabel() == StemperFiConstants.TARGET_ZONE_C) {
                            targetZone = StemperFiConstants.TARGET_ZONE_C;
                        }
                        //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                }
                telemetry.addData("TargetZone: ", targetZone);
                telemetry.update();
            }
            // wobble grabber
            wobbleGrabber(gamepad2.right_trigger, gamepad2.left_stick_x);
        }

        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (tfod != null) {
            tfod.shutdown();
        }

        switch (targetZone) {
            case StemperFiConstants.TARGET_ZONE_A:
                targetZoneRight = 37;
                targetZoneForward = 40;
                targetZonePark = 0;
                break;
            case StemperFiConstants.TARGET_ZONE_B:
                targetZoneRight = -25;
                targetZoneForward = 110;
                targetZonePark = 80;
                break;
            case StemperFiConstants.TARGET_ZONE_C:
                targetZoneRight = 40;
                targetZoneForward = 165;
                targetZonePark = 130;
                break;
        }
        telemetry.addData("TargetZone: ", targetZone);
        telemetry.addData("targetZoneRight: ", targetZoneRight);
        telemetry.addData("targetZoneForward: ", targetZoneForward);
        telemetry.addData("targetZonePark: ", targetZonePark);
        telemetry.update();
        runtime.reset();
        moveForwardCM(153.5 + 10, 0.8);
        moveBackwardsCM(14, 0.8);
        wobbleMotor.setTargetPosition(StemperFiConstants.WOBBLE_UP);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(.5);
        flywheelMotorEx.setPower(StemperFiConstants.FLYWHEEL_RUN_AUTO);
        slideRightCM(36.5, 0.5);


        angleMotor.setTargetPosition(StemperFiConstants.ANGLE_SHOOT_AUTO);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(0.9);
        sleep(1000);
        for (int i = 0; i < 3; i++) {
            long startTime = System.currentTimeMillis();
//            do {
//                telemetry.addData("vel: ", flywheelMotorEx.getVelocity());
//                telemetry.update();
//            } while (System.currentTimeMillis() - startTime < 2500);
            sleep(2500);
            triggerServo.setPosition(StemperFiConstants.TRIGGER_SERVO_FIRE);
            sleep(500);
            triggerServo.setPosition(StemperFiConstants.TRIGGER_SERVO_LOAD);
        }
        moveForwardCM(targetZoneForward, 0.6);
        flywheelMotorEx.setPower(StemperFiConstants.FLYWHEEL_STOP);
        //flywheelMotorEx.setVelocity(0);
        slideRightCM(targetZoneRight, 0.6);
        angleMotor.setTargetPosition(0);
        angleMotor.setPower(0.9);
        wobbleMotor.setTargetPosition(StemperFiConstants.WOBBLE_OUT);
        wobbleMotor.setPower(0.5);
        sleep(1000);
        wobbleGrabber(1, 0);
        sleep(1000);
        wobbleMotor.setTargetPosition(StemperFiConstants.WOBBLE_IN);
        wobbleMotor.setPower(0.8);
        sleep(1000);
        moveBackwardsCM(targetZonePark, 0.6);

    }

    private void initRobot() {

        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        drivetrain = new MecanumDrivetrain(driveMotors);

        angleMotor = hardwareMap.get(DcMotor.class, "angle");
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobble");
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
//        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorEx = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeBottom");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        triggerServo = hardwareMap.get(Servo.class, "kicker");
        triggerServo.setPosition(triggerServoPosition);
        wobbleServo = hardwareMap.get(Servo.class, "wobbleGrabber");
        wobbleServo.setPosition(wobbleServoPosition);

    }

    public void wobbleGrabber(double rightTigger, double x) {
        if (rightTigger > 0.2) {
            wobbleServoPosition = StemperFiConstants.WOBBLE_SERVO_OPEN;
        } else if (x > 0.2) {
            wobbleServoPosition += 0.05;
            wobbleServoPosition = Math.min(StemperFiConstants.WOBBLE_SERVO_CLOSE, wobbleServoPosition);
        } else if (x < -0.2) {
            wobbleServoPosition -= 0.05;
            wobbleServoPosition = Math.max(StemperFiConstants.WOBBLE_SERVO_OPEN, wobbleServoPosition);
        }
        wobbleServo.setPosition(wobbleServoPosition);
//        telemetry.addData("x", x);
//        telemetry.addData("wobblePosition", wobbleServoPosition);
    }

    // Move the robot forwards
    public void moveForwardCM(double cm, double power) {
        int ticks = (int) (cm * StemperFiConstants.TICKS_PER_CM);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("fl: ", frontLeft.getCurrentPosition());
            telemetry.addData("fr: ", frontRight.getCurrentPosition());
            telemetry.addData("bl: ", backLeft.getCurrentPosition());
            telemetry.addData("br: ", backRight.getCurrentPosition());
            telemetry.update();
        }
    }

    // Move the robot backwards
    public void moveBackwardsCM(double cm, double power) {
        // backwards is just negative forwards
        moveForwardCM(-cm, power);
    }

    // strafe robot to the left
    public void slideLeftCM(double cm, double power) {
        int ticks = (int) (cm * StemperFiConstants.SLIDE_TICKS_PER_CM);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    // strafe robot to the right
    public void slideRightCM(double cm, double power) {
        // right is just negative left
        slideLeftCM(-cm, power);
    }

    // rotate the robot
    public void turn(int ticks, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}

