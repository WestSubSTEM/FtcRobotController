package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@TeleOp(name = "Lift", group = "Qual")
public class TeleLift extends TeleMeetQual {


    @Override
    public void loop() {

        // Driver
        if (gamepad1.right_trigger > .9 && gamepad1.left_trigger > .0 && bothDriverTriggersReleased) {
            isForward = !isForward;
            if (isForward) {
                ledFront.setColors(StemperFiConstants.DIRECTION_ACTIVE);
               // ledBack.setColors(StemperFiConstants.DIRECTION_INACTIVE);
            } else {
                ledFront.setColors(StemperFiConstants.DIRECTION_INACTIVE);
               // ledBack.setColors(StemperFiConstants.DIRECTION_ACTIVE);
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
            rotation = -gamepad1.left_stick_x * speedLimit;
        } else {
            course = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI / 2;
            velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) * -speedLimit;
            rotation = gamepad1.left_stick_x * speedLimit;
        }
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

        // Servo Grabber
        if (bumperRight.isPressed()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
        } else if (bumperLeft.isPressed()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
        }
        grabberServo.setPosition(grabberServoPosition);

        // Lift Calculations
            float right_stick_y = -gamepad2.right_stick_y;
            if (Math.abs(right_stick_y) > 0.2) {
                liftMotorTarget = liftMotorTarget + Math.round(right_stick_y * 10.0f);
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1.0);
            }

            if (liftMotorTarget < StemperFiConstants.LIFT_TICKS_LOW) {
                angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
                angleServo.setPosition(angleServoPosition);
            }
        // angle
        /*
        if (!isRotating() && liftMotorCurrentPosition >= StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD && liftMotorTarget >= StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD) {
            if (rotateServoPosition == StemperFiConstants.ROTATE_SERVO_FRONT) {
                angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT + StemperFiConstants.ANGLE_SERVO_SCORE_DELTA);
            } else {
                angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT - StemperFiConstants.ANGLE_SERVO_SCORE_DELTA);
            }
        }
*/
        telemetry.addData("lift Target: ", liftMotorTarget);
        telemetry.addData("lift CurPos:", liftMotor.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void stop() {
    }
}

