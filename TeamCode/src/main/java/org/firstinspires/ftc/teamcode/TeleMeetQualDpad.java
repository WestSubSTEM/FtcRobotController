package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@TeleOp(name = "DPad", group = "State")
public class TeleMeetQualDpad extends TeleMeetQual {

    public boolean isFlat = true;
    public Button flatButton = new Button();
    public Button angleButton = new Button();

    @Override
    public void init() {
        super.init();
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
        dpad.input(gamepad2.dpad_left || gamepad2.dpad_right);
        angleButton.input(gamepad2.dpad_up);
        flatButton.input(gamepad2.dpad_down);
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
            isFlat = true;
            angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
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
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_TWO : LOW;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (buttonY.onPress()) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_THREE : MED;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (buttonA.onPress() && !gamepad2.start) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_FIVE : PLATE;
                liftMotor.setTargetPosition(liftMotorTarget);
                liftMotor.setPower(1);
            } else if (buttonB.onPress() && !gamepad2.start) {
                targetFromButton = true;
                liftMotorTarget = buttonLeftStick.isPressed() ? StemperFiConstants.LIFT_TICKS_FOUR : HIGH;
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
                isFlat = true;
                angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
                //angleServo.setPosition(angleServoPosition);
            }
        }

        // angle
//        if (!isRotating() && liftMotorCurrentPosition >= StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD && liftMotorTarget > StemperFiConstants.ROTATE_SERVO_LIFT_THRESHOLD) {
//            if (rotateServoPosition == StemperFiConstants.ROTATE_SERVO_FRONT) {
//                angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT + StemperFiConstants.ANGLE_SERVO_SCORE_DELTA);
//            } else {
//                angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT - StemperFiConstants.ANGLE_SERVO_SCORE_DELTA_REVERSE);
//            }
//        }

        // Servo Grabber
        if (bumperRight.onPress()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
            grabberServo.setPosition(grabberServoPosition);
        } else if (bumperLeft.onPress()) {
            grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
            grabberServo.setPosition(grabberServoPosition);
        }

        if (angleButton.onPress()) {
            if (rotateServoPosition == StemperFiConstants.ROTATE_SERVO_FRONT) {
                angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT + StemperFiConstants.ANGLE_SERVO_SCORE_DELTA;
            } else {
                angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT - StemperFiConstants.ANGLE_SERVO_SCORE_DELTA_REVERSE;
            }
        } else if (flatButton.onPress()) {
            angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
        }
        angleServo.setPosition(angleServoPosition);

        telemetry.addData("lift Target: ", liftMotorTarget);
        telemetry.addData("lift CurPos:", liftMotor.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void stop() {
    }
}

