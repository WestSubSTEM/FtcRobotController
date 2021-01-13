package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;


@Autonomous(name = "Auto", group = "Meet2")
public class AutoMeet2 extends LinearOpMode {

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
    public DcMotor flywheelMotor;
    public DcMotor intakeMotor;

    public Servo triggerServo;
    public double triggerServoPosition = StemperFiConstants.TRIGGER_SERVO_LOAD;

    public Servo wobbleServo;
    public double wobbleServoPosition = StemperFiConstants.WOBBLE_SERVO_OPEN;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        while (!opModeIsActive() && !isStopRequested()) {
            // wobble grabber
            wobbleGrabber(gamepad2.right_trigger, gamepad2.left_stick_x);

            telemetry.addData("Alliance ", false ? "Blue" : "Red");
            telemetry.update();
        }
        runtime.reset();
        moveForwardCM(153.5, 0.6);
        wobbleMotor.setTargetPosition(StemperFiConstants.WOBBLE_UP);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(.5);
        flywheelMotor.setPower(StemperFiConstants.FLYWHEEL_RUN);
        slideRightCM(36.5, 0.5);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setTargetPosition(645);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(0.9);
        for (int i = 0; i < 3; i++) {
            sleep(2500);
            triggerServo.setPosition(StemperFiConstants.TRIGGER_SERVO_FIRE);
            sleep(500);
            triggerServo.setPosition(StemperFiConstants.TRIGGER_SERVO_LOAD);
        }
        moveForwardCM(100, 0.6);
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
        moveBackwardsCM(75, 0.6);
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
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

}

