package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name = "Meet 2 Tele", group = "Meet2")
public class TeleMeet1 extends OpMode {
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    public DcMotorEx liftMotor;
    public int liftMotorTarget = 0;
    public Servo grabberServo, rotateServo;
    private double grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
    private double rotateServoPosition = 0.5;
    private boolean targetFromButton = false;

    private Button buttonA = new Button();
    private Button buttonB = new Button();
    private Button buttonX = new Button();
    private Button buttonY = new Button();
    private Button buttonLeftStick = new Button();

    private Button bumperLeft = new Button();
    private Button bumperRight = new Button();

    private Button buttonZero = new Button();
    private Button dpadUp = new Button();
    private Button dpadDown = new Button();
    private Button buttonTurntableBlue = new Button();
    private Button buttonTurntableRed = new Button();

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

        /*
        encoderServoLeft = hardwareMap.get(Servo.class, "encoderLeft");
        encoderServoRight = hardwareMap.get(Servo.class, "encoderRight");
        encoderServoCenter = hardwareMap.get(Servo.class, "encoderCenter");

        encoderServoLeft.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_LEFT);
        encoderServoRight.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_RIGHT);
        encoderServoCenter.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_CENTER);
*/

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberServo = hardwareMap.get(Servo.class, "pinchy");
        grabberServo.setPosition(grabberServoPosition);

        rotateServo = hardwareMap.get(Servo.class, "rotoPinchy");
        grabberServo.setPosition(grabberServoPosition);
    }

    @Override
    public void loop() {
        // Driver 1
        double speedLimit = 0.6;
        if (gamepad1.right_bumper) {
            speedLimit = 1;
        } else if (gamepad1.left_bumper) {
            speedLimit = 0.25;
        }
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) * speedLimit;
        double rotation = -gamepad1.left_stick_x * speedLimit;

        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);
        //telemetry.addData("course", String.format("%.01f cm", course));
        //telemetry.addData("velocity", String.format("%.01f mm", velocity));


        buttonA.input(gamepad2.a);
        buttonB.input(gamepad2.b);
        buttonX.input(gamepad2.x);
        buttonY.input(gamepad2.y);
        buttonLeftStick.input(gamepad2.left_stick_button);

        bumperLeft.input(gamepad2.left_bumper);
        bumperRight.input(gamepad2.right_bumper);

        // Servo
        if (bumperRight.isPressed()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
        } else if (bumperLeft.isPressed()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
        }
        grabberServo.setPosition(grabberServoPosition);

        double left_stick_x = gamepad2.left_stick_x;
        if (Math.abs(left_stick_x) > 0.1) {
         //   rotateServoPosition = StemperFiConstants.ROTATE_SERVO_LIMIT * left_stick_x + StemperFiConstants.ROTATE_SERVO_CENTER;
        } else {
          //  rotateServoPosition = StemperFiConstants.ROTATE_SERVO_CENTER;
        }
        rotateServo.setPosition(rotateServoPosition);

        float right_stick_y = -gamepad2.right_stick_y;
        if (buttonX.isPressed()) {
            targetFromButton = true;
            liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_THREE : StemperFiConstants.LIFT_TICKS_LOW;
            liftMotor.setTargetPosition(liftMotorTarget);
            liftMotor.setPower(1);
        } else if (buttonY.isPressed()) {
            targetFromButton = true;
            liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_FOUR : StemperFiConstants.LIFT_TICKS_MED;
            liftMotor.setTargetPosition(liftMotorTarget);
            liftMotor.setPower(1);
        } else if (buttonA.isPressed() && !gamepad2.start) {
            targetFromButton = true;
            liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_TWO : StemperFiConstants.LIFT_TICKS_PLATE;
            liftMotor.setTargetPosition(liftMotorTarget);
            liftMotor.setPower(1);
        } else if (buttonB.isPressed() && !gamepad2.start) {
            targetFromButton = true;
            liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_FIVE : StemperFiConstants.LIFT_TICKS_HIGH;
            liftMotor.setTargetPosition(liftMotorTarget);
            liftMotor.setPower(1);
        } if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) {
            targetFromButton = true;
            liftMotorTarget = 0;
            liftMotor.setTargetPosition(liftMotorTarget);
            liftMotor.setPower(1);
        } else if (Math.abs(right_stick_y) > 0.2) {
            if (targetFromButton) {
                targetFromButton = false;
                liftMotorTarget = liftMotor.getCurrentPosition();
            }
            liftMotorTarget = liftMotorTarget + Math.round(right_stick_y * 50.0f);
            liftMotorTarget = Math.max(0, liftMotorTarget);
            liftMotorTarget = Math.min(liftMotorTarget, StemperFiConstants.LIFT_TICKS_MAX);
            liftMotor.setTargetPosition(liftMotorTarget);
        }
        telemetry.addData("lift Target: ", liftMotorTarget);
        telemetry.addData("lift CurPos:", liftMotor.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void stop() {
    }
}

