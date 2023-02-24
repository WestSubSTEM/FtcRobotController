package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@TeleOp(name = "State", group = "Qual")
public class TeleMeetQual extends OpMode {
    // LED Lights
    public QwiicLEDStrip ledFront;
    //public QwiicLEDStrip ledBack;

    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;
    public boolean isForward = true;
    public boolean bothDriverTriggersReleased = true;
  //  public DigitalChannel magnetSwitch;

    public DcMotorEx liftMotor;
    public int liftMotorTarget = 0;
    public Servo grabberServo, rotateServo, angleServo, cameraServo;
    public double grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
    public double angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
    public double rotateServoPosition = StemperFiConstants.ROTATE_SERVO_FRONT;
    public double rotateServoTarget = rotateServoPosition;
    public boolean targetFromButton = false;


    public Button buttonA = new Button();
    public Button buttonB = new Button();
    public Button buttonX = new Button();
    public Button buttonY = new Button();
    public Button buttonLeftStick = new Button();
    public Button buttonMagnet = new Button();
    public Button dpad = new Button();

    public Button bumperLeft = new Button();
    public Button bumperRight = new Button();

    public Button buttonZero = new Button();
    public Button dpadUp = new Button();
    public Button dpadDown = new Button();
    public Button buttonTurntableBlue = new Button();
    public Button buttonTurntableRed = new Button();

    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

    @Override
    public void init() {
        // Set up LED Strips
        ledFront = hardwareMap.get(QwiicLEDStrip.class, "led_strip_front");
        ledFront.setBrightness(2);
        ledFront.setColors(StemperFiConstants.DIRECTION_ACTIVE);
//        ledBack = hardwareMap.get(QwiicLEDStrip.class, "led_strip_back");
  //      ledBack.setBrightness(4);
    //    ledBack.setColors(StemperFiConstants.DIRECTION_INACTIVE);


        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(liftMotorTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cameraServo = hardwareMap.get(Servo.class, "camera");
        cameraServo.setPosition(0.15);

        grabberServo = hardwareMap.get(Servo.class, "pinchy");
        grabberServo.setPosition(grabberServoPosition);

        rotateServo = hardwareMap.get(Servo.class, "rotate");
        rotateServo.setPosition(rotateServoPosition);

        angleServo = hardwareMap.get(Servo.class, "angle");
        angleServo.setPosition(angleServoPosition);
    }

    public boolean isRotating() {
        return rotateServoTarget != rotateServoPosition;
    }

    @Override
    public void loop() {

        // Driver
        if (gamepad1.right_trigger > .9 && gamepad1.left_trigger > .0 && bothDriverTriggersReleased) {
            bothDriverTriggersReleased = false;
            isForward = !isForward;
            if (isForward) {
                ledFront.setColors(StemperFiConstants.DIRECTION_ACTIVE);
          //      ledBack.setColors(StemperFiConstants.DIRECTION_INACTIVE);
            } else {
                ledFront.setColors(StemperFiConstants.DIRECTION_INACTIVE);
           //     ledBack.setColors(StemperFiConstants.DIRECTION_ACTIVE);
            }
        } else if (gamepad1.right_trigger < .2 && gamepad1.left_trigger < .2 && !bothDriverTriggersReleased) {
            bothDriverTriggersReleased = true;
        }

        double speedLimit = 0.6;
        if (gamepad1.right_bumper) {
            speedLimit = 1;
        } else if (gamepad1.left_bumper) {
            speedLimit = 0.25;
        }
        double course, velocity, rotation;
        if (isForward) {
            course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 2;
            velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) * speedLimit;
        } else {
            course = -(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 2);
            velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) * speedLimit;
        }
        rotation = -gamepad1.left_stick_x * speedLimit;
        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);
        telemetry.addData("course", String.format("%.01f ", course));
        telemetry.addData("velocity", String.format("%.01f ", velocity));
        telemetry.addData("rotation", String.format("%.01f ", rotation));


        // accessory driver
        buttonA.input(gamepad2.a);
        buttonB.input(gamepad2.b);
        buttonX.input(gamepad2.x);
        buttonY.input(gamepad2.y);
        buttonLeftStick.input(gamepad2.left_stick_button);
        dpad.input(gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right);
        bumperLeft.input(gamepad2.left_bumper);
        bumperRight.input(gamepad2.right_bumper);
        int liftMotorCurrentPosition =  liftMotor.getCurrentPosition();


        // Detect rotate
        if (dpad.onPress() && liftMotorCurrentPosition > StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD) {
            liftMotor.setTargetPosition(liftMotorCurrentPosition);
            liftMotor.setPower(1);
            rotateServoTarget = rotateServoTarget == StemperFiConstants.ROTATE_SERVO_FRONT ? StemperFiConstants.ROTATE_SERVO_BACK : StemperFiConstants.ROTATE_SERVO_FRONT;
        }
        if (isRotating()) {
            angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT);
            double rotateServoDiff = rotateServoTarget - rotateServoPosition;
            if (Math.abs(rotateServoDiff) > StemperFiConstants.ROTATE_SERVO_STEP_SIZE) {
                rotateServoPosition += rotateServoDiff >= 0 ? StemperFiConstants.ROTATE_SERVO_STEP_SIZE : -StemperFiConstants.ROTATE_SERVO_STEP_SIZE;
            } else {
                rotateServoPosition = rotateServoTarget;
            }
            rotateServo.setPosition(rotateServoPosition);
        }

        // Lift Calculations
        if (!isRotating()) {
            float right_stick_y = -gamepad2.right_stick_y;
            if (buttonX.onPress()) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_TWO : StemperFiConstants.LIFT_TICKS_LOW;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (buttonY.onPress()) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_THREE : StemperFiConstants.LIFT_TICKS_MED;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (buttonA.onPress() && !gamepad2.start) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_FIVE : StemperFiConstants.LIFT_TICKS_PLATE;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (buttonB.onPress() && !gamepad2.start) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_FOUR : StemperFiConstants.LIFT_TICKS_HIGH;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            }
            if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8) {
                targetFromButton = true;
                grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
                grabberServo.setPosition(grabberServoPosition);
                liftMotorTarget = 0;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (Math.abs(right_stick_y) > 0.2) {
                if (targetFromButton) {
                    targetFromButton = false;
                    liftMotorTarget = liftMotorCurrentPosition;
                }
                liftMotorTarget = liftMotorCurrentPosition + Math.round(right_stick_y * 500.0f);
                liftMotorTarget = Math.max(0, liftMotorTarget);
                liftMotorTarget = Math.min(liftMotorTarget, StemperFiConstants.LIFT_TICKS_MAX);
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            }

            if (liftMotorTarget < StemperFiConstants.LIFT_TICKS_LOW) {
                angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
                angleServo.setPosition(angleServoPosition);
            }
        }
        // angle
        if (!isRotating() && liftMotorCurrentPosition >= StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD && liftMotorTarget > StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD) {
            if (rotateServoPosition == StemperFiConstants.ROTATE_SERVO_FRONT) {
                angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT + StemperFiConstants.ANGLE_SERVO_SCORE_DELTA);
            } else {
                angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT - StemperFiConstants.ANGLE_SERVO_SCORE_DELTA_REVERSE);
            }
        }

        // Servo Grabber
        if (bumperRight.onPress()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
            grabberServo.setPosition(grabberServoPosition);
        } else if (bumperLeft.onPress()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
            grabberServo.setPosition(grabberServoPosition);
        }



        telemetry.addData("lift Target: ", liftMotorTarget);
        telemetry.addData("lift CurPos:", liftMotor.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void stop() {
    }
}

