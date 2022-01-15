package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Arrays;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@TeleOp(name = "Meet 2 Tele", group = "Meet2")
public class TeleMeet1 extends OpMode {
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

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

    private Button buttonA = new Button();
    private Button buttonB = new Button();
    private Button buttonX = new Button();
    private Button buttonY = new Button();
    private Button buttonZero = new Button();
    private Button dpadUp = new Button();
    private Button dpadDown = new Button();
    private Button buttonTurntableBlue = new Button();
    private Button buttonTurntableRed = new Button();

    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

    public int turntableSpeedTicksPerSec = StemperFiConstants.TURNTABLE_MOTOR_INITIAL_SPEED;
    public int ledCount = 3;

    @Override
    public void init() {

        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);

        encoderServoLeft = hardwareMap.get(Servo.class, "encoderLeft");
        encoderServoRight = hardwareMap.get(Servo.class, "encoderRight");
        encoderServoCenter = hardwareMap.get(Servo.class, "encoderCenter");

        encoderServoLeft.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_LEFT);
        encoderServoRight.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_RIGHT);
        encoderServoCenter.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_CENTER);

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
    }

    @Override
    public void loop() {
        // Driver 1
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double rotation = -gamepad1.left_stick_x;

        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);
        //telemetry.addData("course", String.format("%.01f cm", course));
        //telemetry.addData("velocity", String.format("%.01f mm", velocity));

        buttonA.input(gamepad2.a);
        buttonB.input(gamepad2.b);
        buttonX.input(gamepad2.x);
        buttonY.input(gamepad2.y);
        dpadUp.input(gamepad2.dpad_up);
        dpadDown.input(gamepad2.dpad_down);
        buttonZero.input(zeroTouchSensor.isPressed());
        buttonTurntableBlue.input(gamepad2.right_bumper);
        buttonTurntableRed.input(gamepad2.left_bumper);

        if (buttonZero.onPress()) {
            telemetry.addData("ZeroButton Pressed! ", buttonZero.onPress());
            zeroLED.enableLight(true);
            wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wormMotor.setTargetPosition(0);
            wormMotor.setPower(0);
        } else {
            zeroLED.enableLight(false);
        }

        if (buttonX.onPress()) {
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INTAKE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_INTAKE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (buttonY.onPress()) {
            bucketServoRightPosition =StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_TOP);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (buttonB.onPress()) {
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_MIDDLE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_MIDDLE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (buttonA.onPress()) {
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_BOTTOM;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_BOTTOM);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setWormMotor(gamepad2.left_stick_y);
            setBucket(gamepad2.right_stick_y);
        }

        setIntake(gamepad2.right_trigger, gamepad2.left_trigger);
        if (buttonTurntableBlue.onPress()) {
                topLEDStrip.setColors(getLedArray());
                turntableMotor.setVelocity(turntableSpeedTicksPerSec);
        }
        if (buttonTurntableBlue.isPressed() && dpadUp.onPress()) {
            ledCount++;
            topLEDStrip.setColors(getLedArray());
            turntableSpeedTicksPerSec += StemperFiConstants.TURNTABLE_INCREMENT;
            turntableMotor.setVelocity(turntableSpeedTicksPerSec);
        }
        if (buttonTurntableBlue.isPressed() && dpadDown.onPress()) {
            ledCount--;
            topLEDStrip.setColors(getLedArray());
            turntableSpeedTicksPerSec -= StemperFiConstants.TURNTABLE_INCREMENT;
            turntableMotor.setVelocity(turntableSpeedTicksPerSec);
        }
        if (buttonTurntableBlue.onRelease()) {
            topLEDStrip.setColor(StemperFiConstants.RED);
            turntableMotor.setVelocity(0);
        }

        if (buttonTurntableRed.onPress()) {
            topLEDStrip.setColors(getLedArray());
            turntableMotor.setVelocity(-turntableSpeedTicksPerSec);
        }
        if (buttonTurntableRed.isPressed() && dpadUp.onPress()) {
            ledCount++;
            topLEDStrip.setColors(getLedArray());
            turntableSpeedTicksPerSec += StemperFiConstants.TURNTABLE_INCREMENT;
            turntableMotor.setVelocity(-turntableSpeedTicksPerSec);
        }
        if (buttonTurntableRed.isPressed() && dpadDown.onPress()) {
            ledCount--;
            topLEDStrip.setColors(getLedArray());
            turntableSpeedTicksPerSec -= StemperFiConstants.TURNTABLE_INCREMENT;
            turntableMotor.setVelocity(-turntableSpeedTicksPerSec);
        }
        if (buttonTurntableRed.onRelease()) {
            topLEDStrip.setColor(StemperFiConstants.RED);
            turntableMotor.setVelocity(0);
        }

        telemetry.addData("b right: ", bucketServoRightPosition);
        telemetry.addData("b  left: ", bucketServoLeftPosition);
        telemetry.addData("turntableSpeedTicksPerSec: ", turntableSpeedTicksPerSec);
        telemetry.addData("turntablePower: ", turntableMotor.getPower());
        telemetry.addData("dpadUP: ", dpadUp.isPressed());
        telemetry.addData("dpadDown: ", dpadDown.isPressed());

        telemetry.update();
    }

    private int[] getLedArray() {
        if (ledCount > 9) {
            ledCount = 1;
        } else if (ledCount == 0) {
            ledCount = 10;
        }
        int[] result = new int[10];
        Arrays.fill(result, StemperFiConstants.OFF);
        Arrays.fill(result, 0, ledCount - 1, StemperFiConstants.GREEN);
        return result;
    }

    private void setBucket(double rightStickY) {
        if (rightStickY > 0.2 || rightStickY < -0.2) {
            if (rightStickY > 0.2) {
                bucketServoRightPosition += 0.005;
            } else {
                bucketServoRightPosition -= 0.005;
            }
            bucketServoRightPosition = Math.max(bucketServoRightPosition, StemperFiConstants.BUCKET_SERVO_RIGHT_MIN);
            bucketServoRightPosition = Math.min(bucketServoRightPosition, StemperFiConstants.BUCKET_SERVO_RIGHT_MAX);
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
        }
    }

    private void setWormMotor(double leftStickY) {

        int targetPosition = wormMotor.getTargetPosition();
        if (leftStickY > 0.2 || leftStickY < -0.2) {
            if (leftStickY > 0.2 && !buttonZero.isPressed()) {
                targetPosition = wormMotor.getCurrentPosition() + 100 ;
            } else if (leftStickY <  -0.2) {
                targetPosition = wormMotor.getCurrentPosition() - 100;
            }
        }
        wormMotor.setTargetPosition(targetPosition);
        wormMotor.setPower(1);
        wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("WormTarget: ", targetPosition);

    }

    private void setIntake(double rightTrigger, double leftTrigger) {
        if (rightTrigger > 0.1) {
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_IN;
        } else if (leftTrigger > 0.1) {
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
        } else {
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;
        }
        intakeServo.setPosition(intakeServoSpeed);
        telemetry.addData("Intake Servo: ", intakeServoSpeed);
    }


    @Override
    public void stop() {
        topLEDStrip.turnAllOff();
    }
}

