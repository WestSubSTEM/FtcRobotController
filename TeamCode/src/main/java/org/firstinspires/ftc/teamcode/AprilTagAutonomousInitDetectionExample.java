/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@Autonomous(name = "Auto Tag", group = "Qual")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    private Button bumperLeft = new Button();
    private Button bumperRight = new Button();
    private Button increaseSecButton = new Button();
    private Button decreaseSecButton = new Button();
    private Button redButton = new Button();
    private Button blueButton = new Button();

    private ElapsedTime runtime = new ElapsedTime();
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor leftEncoder;
    public DcMotor frontRight;
    public DcMotor rightEncoder;
    public DcMotor backLeft;
    public DcMotor backEncoder;
    public DcMotor backRight;
    public DcMotor[] driveMotors;
    // The MecanumDrivetrain courteous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;

    public DcMotorEx liftMotor;
    public int liftMotorTarget = 0;
    public Servo grabberServo, rotateServo, angleServo;
    public double grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
    public double angleServoPosition = StemperFiConstants.ANGLE_SERVO_FLAT;
    public double rotateServoPosition = StemperFiConstants.ROTATE_SERVO_FRONT;
    public double rotateServoTarget = rotateServoPosition;
    public boolean isRight = true;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST_1 = 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST_2 = 2; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST_3 = 2; // Tag ID 1 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    int delaySeconds = 0;

    private void rotateCW(int degrees, double power) {
        rotateCCW(degrees, -power);
    }

    private void rotateCCW(int degrees, double power) {
        int ticks = (20_000 / 90) * degrees;
        stopAndReset();
        backRight.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        long cpMax = 0;
        long cpr = 0;
        long cpl = 0;
        long cpb = 0;
        do {
            cpr = Math.abs(rightEncoder.getCurrentPosition());
            cpl = Math.abs(leftEncoder.getCurrentPosition());
            cpb = Math.abs(backEncoder.getCurrentPosition());
            cpMax = Math.max(cpr, cpl);
            telemetry.addData("bw   cpr: ", cpr);
            telemetry.addData("bw   cpl: ", cpl);
            telemetry.addData("bw  back:", cpb);
            telemetry.addData("bw cpMax: ", cpMax);
            telemetry.addData("bw    tp: ", ticks);
            telemetry.update();
        } while (cpMax < ticks && opModeIsActive());
        stopAndReset();

    }

    private void initRobot() {

        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);
        stopAndReset();

        rightEncoder = backRight;
        leftEncoder = backLeft;
        backEncoder = frontRight;

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(liftMotorTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberServo = hardwareMap.get(Servo.class, "pinchy");
        grabberServo.setPosition(grabberServoPosition);

        rotateServo = hardwareMap.get(Servo.class, "rotate");
        rotateServo.setPosition(rotateServoPosition);

        angleServo = hardwareMap.get(Servo.class, "angle");
        angleServo.setPosition(angleServoPosition);
    }

    public void stopAndReset() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void moveForwardMM(long mm, double power) {
        long ticks = mm * StemperFiConstants.TICKS_PER_MM;
        stopAndReset();
        backLeft.setPower(power);
        frontLeft.setPower(power);

        frontRight.setPower(power * 0.6);
        backRight.setPower(power * 0.6);
        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }
        long cpMax = 0;
        long cpr = 0;
        long cpl = 0;
        long cpb = 0;
        do {
            cpr = Math.abs(rightEncoder.getCurrentPosition());
            cpl = Math.abs(leftEncoder.getCurrentPosition());
            cpb = Math.abs(backEncoder.getCurrentPosition());
            cpMax = Math.max(cpr, cpl);
            telemetry.addData("bw   cpr: ", cpr);
            telemetry.addData("bw   cpl: ", cpl);
            telemetry.addData("bw  back:", cpb);
            telemetry.addData("bw cpMax: ", cpMax);
            telemetry.addData("bw    tp: ", ticks);
            telemetry.update();
        } while (cpMax < ticks && opModeIsActive());
        stopAndReset();
    }

    // strafe robot to the left
    public void slideLeftTime(int miliseconds, double power) {
        stopAndReset();

        frontRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);

        sleep(miliseconds);

        stopAndReset();
    }

    // strafe robot to the right
    public void slideRightTime(int miliseconds, double power) {
        stopAndReset();

        frontRight.setPower(-power);
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);

        sleep(miliseconds);

        stopAndReset();
    }


    @Override
    public void runOpMode()
    {
        initRobot();
        stopAndReset();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            bumperLeft.input(gamepad2.left_bumper);
            bumperRight.input(gamepad2.right_bumper);

            // Servo
            if (bumperRight.isPressed()) {
                grabberServoPosition = StemperFiConstants.GRABBER_SERVO_CLOSED;
            } else if (bumperLeft.isPressed()) {
                grabberServoPosition = StemperFiConstants.GRABBER_SERVO_OPEN;
            }
            grabberServo.setPosition(grabberServoPosition);

            increaseSecButton.input(gamepad2.y);
            decreaseSecButton.input(gamepad2.a);
            if (increaseSecButton.onPress())  {
                delaySeconds++;
            }
            if (decreaseSecButton.onPress()) {
                delaySeconds--;
            }
            delaySeconds = Math.max(0, delaySeconds);
            delaySeconds = Math.min(20, delaySeconds);
            telemetry.addData("Delay Seconds: ", delaySeconds);

            if (redButton.onPress()) {
                isRight = false;
            }
            if (blueButton.onPress()) {
                isRight = true;
            }
            if (isRight) {
                telemetry.addLine("Left Side of Field");
            } else {
                telemetry.addLine("Right Side of Field ");
            }

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id < 4)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }

        if (delaySeconds > 0) {
            sleep(delaySeconds * 1_000);
        }
        /* Actually do something useful */
        if(tagOfInterest != null && tagOfInterest.id == 1) {
            slideLeftTime(2_000, .4);
            rotateCCW(5, .4);
            moveForwardMM(900, .4);
            slideRightTime(500, .2);
        } else if (tagOfInterest != null && tagOfInterest.id == 3) {
            slideRightTime(1_700, .4);
            rotateCW(3, .4);
            moveForwardMM(900, .4);
        } else {
            moveForwardMM(900, .4);
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
