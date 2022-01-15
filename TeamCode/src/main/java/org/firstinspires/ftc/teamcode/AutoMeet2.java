package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@Autonomous(name = "Cube", group = "Meet2")
public class AutoMeet2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx turntableMotor, wormMotor;
    public TouchSensor zeroTouchSensor;
    public LED zeroLED;

    public int[] turntableLEDS = new int[10];
    public int turntableLightCount = 6;

    public QwiicLEDStrip topLEDStrip;

    public Servo intakeServo;
    public double intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;

    //public Servo bucketServo;
    //public double bucketServoPosition = StemperFiConstants.BUCKET_SERVO_INIT;
    //public DcMotorEx bucketMotor;
    //public int bucketMotorPosition = 0;

    private Servo encoderServoRight, encoderServoLeft, encoderServoCenter;
    private Servo bucketServoLeft, bucketServoRight;
    private double bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INIT;
    private double bucketServoLeftPosition = 1 - bucketServoRightPosition;

    private int MM_TO_TOWER = 410;

    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor leftEncoder;
    public DcMotor frontRight;
    public DcMotor rightEncoder;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;
    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

    public Button redButton = new Button();
    public Button blueButton =new Button();
    public Button greenButton = new Button();
    public Button dPadUpButton = new Button();
    public Button dPadDownButton = new Button();

    public boolean isBlue = true;
    public int wait = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        stopAndReset();
//        boolean dingo = true;
//        while(dingo) {
//            telemetry.addData("left: ", frontLeft.getCurrentPosition());
//            telemetry.addData("right: ", frontRight.getCurrentPosition());
//            telemetry.update();
//        }


        telemetry.addData("Waiting for Start!", "!!!!");
        telemetry.update();
        waitForStart();

        if (wait > 0) {
            sleep(1_000 * wait);
        }
        /*
        turnRight(1000, .4);
        while (opModeIsActive()) {
            sleep(100);
        }*/
        if (opModeIsActive()) {
            moveBackwardsMM(MM_TO_TOWER, .4);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_TOP);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            intakeServo.setPosition(intakeServoSpeed);

            sleep(1000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;
            intakeServo.setPosition(intakeServoSpeed);

            moveForwardMM(MM_TO_TOWER / 2, .4);


            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INIT;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_INTAKE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(opModeIsActive()) {
            if (isBlue) {
                slideRightTime(1000, .6);
            } else {
                slideLeftTime(1000, .6);
            }
        }
    }

    private void initRobot() {

        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftEncoder = frontLeft;
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        rightEncoder = frontRight;
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);

        encoderServoLeft = hardwareMap.get(Servo.class, "encoderLeft");
        encoderServoRight = hardwareMap.get(Servo.class, "encoderRight");
        encoderServoCenter = hardwareMap.get(Servo.class, "encoderCenter");

        encoderServoLeft.setPosition(StemperFiConstants.ENCODER_SERVO_AUT0_LEFT);
        encoderServoRight.setPosition(StemperFiConstants.ENCODER_SERVO_AUT0_RIGHT);
        encoderServoCenter.setPosition(StemperFiConstants.ENCODER_SERVO_AUT0_CENTER);

        turntableMotor = hardwareMap.get(DcMotorEx.class, "turntable");
        turntableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLEDStrip = hardwareMap.get(QwiicLEDStrip.class, "TopLED");
        topLEDStrip.turnAllOff();
        topLEDStrip.setBrightness(1);

        wormMotor = hardwareMap.get(DcMotorEx.class, "worm");
        wormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setTargetPosition(0);
        wormMotor.setPower(0.5);

        intakeServo = hardwareMap.get(Servo.class, "intake");
        intakeServo.setPosition(StemperFiConstants.INTAKE_SERVO_SPEED_OFF);

        bucketServoLeft = hardwareMap.get(Servo.class, "Left_Bucket_Servo");
        bucketServoRight = hardwareMap.get(Servo.class, "Right_Bucket_Servo");
        bucketServoLeft.setPosition(bucketServoLeftPosition);
        bucketServoRight.setPosition(bucketServoRightPosition);

        //bucketServo = hardwareMap.get(Servo.class, "bucket");
        //bucketServo.setPosition(bucketServoPosition);

        zeroTouchSensor = hardwareMap.get(TouchSensor.class, "zero");
        zeroLED = hardwareMap.get(LED.class, "zeroLed");

        do {
            greenButton.input(gamepad2.a);
            blueButton.input(gamepad2.x);
            redButton.input(gamepad2.b);
            dPadUpButton.input(gamepad2.dpad_up);
            dPadDownButton.input(gamepad2.dpad_down);
            if (redButton.onPress()) {
                isBlue = false;
            }
            if (blueButton.onPress()) {
                isBlue = true;
            }
            if (dPadUpButton.onPress()) {
                wait++;
            }
            if (dPadDownButton.onPress()) {
                wait--;
                wait = Math.max(wait, 0);
            }
            telemetry.addData("color: ", isBlue ? "Blue" : "Red");
            telemetry.addData("wait: ", wait);
            telemetry.addData("Press Green To Arm", "!");
            telemetry.update();
        } while (!greenButton.isPressed());


    }

    public void turnRight(int degrees, double power) {
        stopAndReset();
        frontRight.setPower(-power);
        backRight.setPower(-power);
        frontLeft.setPower(power);
        backLeft.setPower(power);

        long ticks = StemperFiConstants.TICKS_PER_DEGREE * degrees;

        long cp = 0;
        do {
            long rcp =rightEncoder.getCurrentPosition();
            long lcp =leftEncoder.getCurrentPosition();
            cp = rcp;
            telemetry.addData("tr rcp cp: ", rcp);
            telemetry.addData("tr lcp cp: ", lcp);
            telemetry.addData("tr le tp: ", ticks);
            telemetry.update();
        } while (cp > ticks);
        stopAndReset();
    }

    public void turnLeft(int degrees, double power) {
        stopAndReset();
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        long ticks = StemperFiConstants.TICKS_PER_DEGREE * degrees;

        long cp = 0;
        do {
            long rcp =rightEncoder.getCurrentPosition();
            long lcp =leftEncoder.getCurrentPosition();
            cp = rcp;
            telemetry.addData("tr rcp cp: ", rcp);
            telemetry.addData("tr lcp cp: ", lcp);
            telemetry.addData("tr le tp: ", ticks);
            telemetry.update();
        } while (cp > ticks);
        stopAndReset();
    }


    public void moveForwardMM(long mm, double power) {
        stopAndReset();
        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }
        long cp = 0;
        long ticks = StemperFiConstants.TICKS_PER_MM * mm;
        do {
            cp = rightEncoder.getCurrentPosition();
            telemetry.addData("r cp: ", cp);
            telemetry.addData("r tp: ", ticks);
            telemetry.update();
        } while (cp < ticks && opModeIsActive());
        stopAndReset();
    }

    public void moveBackwardsMM(long mm, double power) {
        long ticks = mm * StemperFiConstants.TICKS_PER_MM * -1;
        stopAndReset();;
        for (DcMotor motor : driveMotors) {
            motor.setPower(-power);
        }
        long cp = 0;
        do {
            cp = rightEncoder.getCurrentPosition();
            telemetry.addData("bw cp: ", cp);
            telemetry.addData("bw tp: ", ticks);
            telemetry.update();
        } while (cp > ticks && opModeIsActive());
        stopAndReset();
    }

    public void stopAndReset() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // Move the robot forwards
    public void moveForwardTime(long timems, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        }
        sleep(timems);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(0);
        }
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
    public void slideLeftTime(int miliseconds, double power) {
        stopAndReset();

        frontRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);

        sleep(miliseconds);

        stopAndReset();
    }

    // strafe robot to the right
    public void slideRightTime(int miliseconds, double power) {
        stopAndReset();

        frontRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);

        sleep(miliseconds);

        stopAndReset();
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

}