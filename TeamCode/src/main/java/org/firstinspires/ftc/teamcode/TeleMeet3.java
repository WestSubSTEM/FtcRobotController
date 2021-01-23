package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@TeleOp(name = "Teleop", group = "Meet3")
public class TeleMeet3 extends OpMode {
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    public DcMotor angleMotor;
    public int anglePosition = 0;
    public DcMotor wobbleMotor;
    public int wobbleArmPosition = 0;
    public DcMotor flywheelMotor;
    public DcMotor intakeMotor;

    public Servo triggerServo;
    public double triggerServoPosition = StemperFiConstants.TRIGGER_SERVO_LOAD;

    public Servo wobbleServo;
    public double wobbleServoPosition = StemperFiConstants.WOBBLE_SERVO_OPEN;

    private Button buttonA = new Button();
    private Button buttonB = new Button();
    private Button buttonY = new Button();
    private Button buttonX = new Button();
    private Button buttonRightBumper = new Button();
    private Button buttonStart = new Button();

    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {

        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);

        angleMotor = hardwareMap.get(DcMotor.class, "angle");
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobble");
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeBottom");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        triggerServo = hardwareMap.get(Servo.class, "kicker");
        triggerServo.setPosition(triggerServoPosition);
        wobbleServo = hardwareMap.get(Servo.class, "wobbleGrabber");
        wobbleServo.setPosition(wobbleServoPosition);
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        // Driver 1
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double rotation = -gamepad1.left_stick_x;

        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);
        telemetry.addData("course", String.format("%.01f cm", course));
        telemetry.addData("velocity", String.format("%.01f mm", velocity));

        // Flywheel
        buttonA.input(gamepad2.a);
        buttonB.input(gamepad2.b);
        buttonStart.input(gamepad2.start);
        if (buttonA.isPressed() && !buttonStart.isPressed()) {
            flywheelMotor.setPower(StemperFiConstants.FLYWHEEL_RUN);
        }
        if (buttonB.isPressed() && !buttonStart.isPressed()) {
            flywheelMotor.setPower(StemperFiConstants.FLYWHEEL_STOP);
        }
        telemetry.addData("flywheel", flywheelMotor.getPower());

        // intake
        setIntake(-gamepad2.left_stick_y);

        // shoot
        shoot(gamepad2.left_trigger);

        // wobble arm
        wobble(gamepad2.right_stick_x);
        // wobble grabber
        wobbleGrabber(gamepad2.right_trigger, gamepad2.left_stick_x);

        // platform angle
        buttonX.input(gamepad2.x);
        buttonY.input(gamepad2.y);
        buttonRightBumper.input(gamepad2.right_bumper);
        angle(-gamepad2.right_stick_y, buttonX, buttonY, buttonRightBumper);
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

    public void angle(double y, Button bX, Button bY, Button bRB) {
        if (bRB.isPressed()) {
            anglePosition = StemperFiConstants.ANGLE_POWER_SHOT;
        } else if (bY.isPressed()) {
            anglePosition = StemperFiConstants.ANGLE_SHOOT;
        } else if (bX.isPressed()) {
            anglePosition = StemperFiConstants.ANGLE_DOWN;
        } else if (y > 0.2) {
            anglePosition = anglePosition + Math.round((float) (2 * y));
            anglePosition = Math.min(anglePosition, 720);
        } else if (y < -0.2) {
            anglePosition = anglePosition + Math.round((float) (2 * y));
            anglePosition = Math.max(anglePosition, 0);
        }
        angleMotor.setTargetPosition(anglePosition);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(1);
//        else {
//            anglePosition = angleMotor.getCurrentPosition();
//            angleMotor.setTargetPosition(anglePosition);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(1);
//        }
        telemetry.addData("angle", angleMotor.getCurrentPosition());
        telemetry.addData("angle power", angleMotor.getPower());
    }

    public void wobble(double x) {
        if (x > 0.2) {
            wobbleArmPosition = wobbleArmPosition - Math.round((float) (15 * x));
            wobbleArmPosition = Math.max(wobbleArmPosition, StemperFiConstants.WOBBLE_OUT);
            wobbleMotor.setTargetPosition(wobbleArmPosition);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.9);
        } else if (x < -0.2) {
            wobbleArmPosition = wobbleArmPosition - Math.round((float) (15 * x));
            wobbleArmPosition = Math.min(wobbleArmPosition, StemperFiConstants.WOBBLE_IN);
            wobbleMotor.setTargetPosition(wobbleArmPosition);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.9);
        } else {
            wobbleMotor.setTargetPosition(wobbleArmPosition);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.9);
        }
        telemetry.addData("wobble", wobbleMotor.getCurrentPosition());
    }

    public void shoot(double trigger) {
        if (trigger > 0.2) {
            triggerServoPosition = StemperFiConstants.TRIGGER_SERVO_LOAD - ((StemperFiConstants.TRIGGER_SERVO_LOAD - StemperFiConstants.TRIGGER_SERVO_FIRE) * trigger);
        } else {
            triggerServoPosition = StemperFiConstants.TRIGGER_SERVO_LOAD;
        }
        triggerServo.setPosition(triggerServoPosition);
    }

    public void setIntake(double intakeSpeed) {
        if (Math.abs(intakeSpeed) > 0.2) {
            intakeMotor.setPower(StemperFiConstants.INTAKE_MOTOR_IN * intakeSpeed);
        } else {
            intakeMotor.setPower(StemperFiConstants.INTAKE_MOTOR_STOP);
        }
    }
}

