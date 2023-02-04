package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;
@Disabled
@TeleOp(name = "Drive Only", group = "Backup")
public class Drive extends OpMode {
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;
    public DcMotorEx liftMotor;
    //public DigitalChannel magnetSwitch;
    public Servo grabberServo, rotateServo, angleServo;
    private double grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
    private double angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
    private double rotateServoPosition = StemperFiConstants.ROTATE_SERVO_FRONT;

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

        // get a reference to our digitalTouch object.
        //magnetSwitch = hardwareMap.get(DigitalChannel.class, "magnet");

        // set the digital channel to input.
        //magnetSwitch.setMode(DigitalChannel.Mode.INPUT);


        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        grabberServo = hardwareMap.get(Servo.class, "pinchy");
        grabberServo.setPosition(grabberServoPosition);

        rotateServo = hardwareMap.get(Servo.class, "rotate");
        rotateServo.setPosition(rotateServoPosition);

        angleServo = hardwareMap.get(Servo.class, "angle");
        angleServo.setPosition(angleServoPosition);

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        // Driver 1
        double speedLimit = 0.75;
        if (gamepad1.right_bumper) {
            speedLimit = 1;
        }
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) * speedLimit;
        double rotation = -gamepad1.left_stick_x * speedLimit;

        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);

        float right_stick_y = -gamepad2.right_stick_y;
        if (Math.abs(right_stick_y) > 0.1) {
            liftMotor.setPower(.5 * right_stick_y);
        } else {
            liftMotor.setPower(0);
        }
        telemetry.addData("course", String.format("%.01f cm", course));
        telemetry.addData("velocity", String.format("%.01f mm", velocity));
        telemetry.addData("lift", liftMotor.getCurrentPosition());
       // telemetry.addData("mag", !magnetSwitch.getState());
        /*
        telemetry.addData("fl", frontLeft.getCurrentPosition());
        telemetry.addData("fr", frontRight.getCurrentPosition());
        telemetry.addData("bl", backLeft.getCurrentPosition());
        telemetry.addData("br", backRight.getCurrentPosition());
         */
        telemetry.update();
    }
}