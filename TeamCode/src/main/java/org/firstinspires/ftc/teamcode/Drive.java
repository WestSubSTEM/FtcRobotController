package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    }
}