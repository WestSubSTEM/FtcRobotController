package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LaChouBot Odom", group="FTC Lib")
//@Disabled
public class LaChouOdo extends LinearOpMode {
    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 10.25;
    public static final double CENTER_WHEEL_OFFSET = -2;
    public static final double WHEEL_DIAMETER = 1.89;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    MecanumDrive mecanum;
    GamepadEx driverOp;
    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);

        Motor frontLeft = new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312);

        // mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);


//        leftEncoder = new MotorEx(hardwareMap, "left odometer");
        leftEncoder = new MotorEx(hardwareMap, "frontright");
        //rightEncoder = new MotorEx(hardwareMap, "right odometer");
        rightEncoder = new MotorEx(hardwareMap, "backleft");
        //perpEncoder = new MotorEx(hardwareMap, "center odometer");
        perpEncoder = new MotorEx(hardwareMap, "frontleft");


        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        // read the current position from the position tracker
        odometry.updatePose(PositionTracker.robotPose);

        telemetry.addData("Robot Position at Init: ", PositionTracker.robotPose);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            telemetry.addData("heading rad", PositionTracker.robotPose.getHeading());
            telemetry.addData("heading deg", PositionTracker.robotPose.getHeading() * 57.2958);
            telemetry.addData("pos, ", PositionTracker.robotPose.toString());
            double lx = driverOp.getLeftX();
            double ly = driverOp.getLeftY();
            double rx = driverOp.getRightX();

        mecanum.driveRobotCentric(
                lx,
                ly,
                rx,
                false
        );
            telemetry.addData("Left X", lx);
            telemetry.addData("Left Y", ly);
            telemetry.addData("Right X", rx);
            // teleop things

            // update position
            telemetry.update();
        }
    }
}
