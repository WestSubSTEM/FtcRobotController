package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@TeleOp(name = "Meet 1 Tele", group = "Meet1")
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

    public Servo intakeServo;
    public double intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;

    public Servo bucketServo;
    public double bucketServoPosition = StemperFiConstants.BUCKET_SERVO_INIT;

    private Servo encoderServoRight, encoderServoLeft, encoderServoCenter;

    private Button buttonA = new Button();
    private Button buttonB = new Button();
    private Button buttonX = new Button();
    private Button buttonY = new Button();
    private Button buttonZero = new Button();

    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

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

        wormMotor = hardwareMap.get(DcMotorEx.class, "worm");
        wormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setTargetPosition(0);
        wormMotor.setPower(1);
        intakeServo = hardwareMap.get(Servo.class, "intake");
        intakeServo.setPosition(StemperFiConstants.INTAKE_SERVO_SPEED_OFF);

        bucketServo = hardwareMap.get(Servo.class, "bucket");
        bucketServo.setPosition(bucketServoPosition);

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
        buttonZero.input(zeroTouchSensor.isPressed());

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
            bucketServoPosition = StemperFiConstants.BUCKET_SERVO_INTAKE;
            bucketServo.setPosition(bucketServoPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_INTAKE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (buttonY.onPress()) {
            bucketServoPosition =StemperFiConstants.BUCKET_SERVO_GOAL_TOP;
            bucketServo.setPosition(bucketServoPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_TOP);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (buttonB.onPress()) {
            bucketServoPosition = StemperFiConstants.BUCKET_SERVO_GOAL_MIDDLE;
            bucketServo.setPosition(bucketServoPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_MIDDLE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (buttonA.onPress()) {
            bucketServoPosition = StemperFiConstants.BUCKET_SERVO_GOAL_BOTTOM;
            bucketServo.setPosition(bucketServoPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_BOTTOM);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setWormMotor(gamepad2.left_stick_y);
            setBucket(gamepad2.right_stick_y);
        }

        setIntake(gamepad2.right_trigger, gamepad2.left_trigger);
        setTurntable(gamepad2.right_bumper);
        telemetry.update();
    }

    private void setBucket(double rightStickY) {
        if (rightStickY > 0.2 || rightStickY < -0.2) {
            if (rightStickY > 0.2) {
                bucketServoPosition += 0.005;
            } else {
                bucketServoPosition -= 0.005;
            }
        }
        bucketServoPosition = Math.min(StemperFiConstants.BUCKET_SERVO_MAX, bucketServoPosition);
        bucketServoPosition = Math.max(StemperFiConstants.BUCKET_SERVO_MIN, bucketServoPosition);
        bucketServo.setPosition(bucketServoPosition);
        telemetry.addData("Bucket: ", bucketServoPosition);
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

    private void setTurntable(boolean rightBumper) {
        if (rightBumper) {
            turntableMotor.setPower(StemperFiConstants.TURNTABLE_MOTOR_ON);
        } else {
            turntableMotor.setPower(0);
        }
        telemetry.addData("Turntable On: ", turntableMotor.getPower());
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

}

