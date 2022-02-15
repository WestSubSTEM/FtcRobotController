package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.state.Button;

@Autonomous(name = "Qual Auto", group = "Meet4")
public class AutoMeet4 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx turntableMotor, wormMotor;
    public TouchSensor zeroTouchSensor;
    public LED zeroLED;

    public int[] turntableLEDS = new int[10];
    public int turntableLightCount = 6;

    public QwiicLEDStrip topLEDStrip;

    public Servo intakeServo;
    public double intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;

    //public Servo bucketServo;
    //public double bucketServoPosition = StemperFiConstants.BUCKET_SERVO_INIT;
    //public DcMotorEx bucketMotor;
    //public int bucketMotorPosition = 0;

    private Servo encoderServoRight, encoderServoLeft, encoderServoCenter;
    private Servo bucketServoLeft, bucketServoRight;
    private double bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INIT;
    private double bucketServoLeftPosition = 1 - bucketServoRightPosition;

    private int MM_TO_TOWER_TOP = 450;
    private int MM_TO_TOWER_MIDDLE = 450 - 160;
    private int MM_TO_TOWER_BOTTOM = 450 - 140 ;

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

    public Button redButton = new Button();
    public Button blueButton =new Button();
    public Button greenButton = new Button();
    public Button dPadUpButton = new Button();
    public Button dPadDownButton = new Button();

    public boolean isBlue = true;
    public int wait = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        stopAndReset();
//        boolean dingo = true;
//        while(dingo) {
//            telemetry.addData("left: ", frontLeft.getCurrentPosition());
//            telemetry.addData("right: ", frontRight.getCurrentPosition());
//            telemetry.update();
//        }


        telemetry.addData("Waiting for Start!", "!!!!");
        telemetry.update();
        waitForStart();

        if (wait > 0) {
            sleep(1_000 * wait);
        }

        if (opModeIsActive()) {

            int slideTime = 1600;
            double powSlide = .4;

//SCORING BOTTOM OF HUB AUTO
            /*
            slideLeftTime(slideTime,powSlide);
            moveBackwardsMM(MM_TO_TOWER_BOTTOM, .4);
            bucketServoRightPosition = StemperFiConstants.BOTTOM_SERVO_SCORE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.BOTTOM_WORM_GEAR);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);




            sleep(1000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            intakeServo.setPosition(intakeServoSpeed);

            sleep (1000);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_MIDDLE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);

            intakeServo.setPosition(intakeServoSpeed);
            */

            //SCORING MIDDLE OF HUB AUTO

            slideLeftTime(slideTime,powSlide);
            moveBackwardsMM(MM_TO_TOWER_MIDDLE, .4);
            /*
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_MIDDLE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_MIDDLE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);

            bucketServoRightPosition = StemperFiConstants.MID_SERVO_SCORE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);


            sleep(1000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            intakeServo.setPosition(intakeServoSpeed);

            sleep (1000);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_MIDDLE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);

            intakeServo.setPosition(.5);

                */
            // SCORING TOP OF HUB AUTO
           /* slideLeftTime(slideTime,powSlide);
            moveBackwardsMM(MM_TO_TOWER_TOP, .4);
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_TOP);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);




            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            sleep(2000);
            intakeServo.setPosition(intakeServoSpeed);
*/

            //sleep(4000);
            turnRight( 90, .5);
            slideLeftTime(1600, .6);
    sleep(1000);


            //Moves the robot towards the depot
            moveBackwardsMM(900,-.5);


            //Moves the arm and the servo to the intake position
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_INTAKE+50);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INTAKE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            sleep(2000);

            //Turns the intake on and moves the robot into the blocks
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_IN;
            intakeServo.setPosition(intakeServoSpeed);
            moveForwardMM(800,.3);


            //Spits out any double blocks
            //intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            //intakeServo.setPosition(intakeServoSpeed);
            //sleep(500);
            //Turns the intake off
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;
            intakeServo.setPosition(intakeServoSpeed);

            //Realign the robot by bumping it up against the wall
            slideLeftTime(500, .5);

            //Moves the arm and the servo to the top goal position
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_TOP);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Moves the robot out of the depot aligns it with the team hub
            moveForwardMM(950,-.7);
            slideRightTime(500,.5);
            turnLeft( 32, .5);
            moveForwardMM(460,-.7);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);

            sleep(2000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            intakeServo.setPosition(intakeServoSpeed);


            sleep(2000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;
            intakeServo.setPosition(intakeServoSpeed);

            moveForwardMM(370,.7);
            turnRight(34,.5);
            slideLeftTime(500,.5);
            moveForwardMM(720,.7);

            //Moves the arm and the servo to the intake position
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_INTAKE+50);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INTAKE;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            sleep(2000);

            /*
            slideRightTime(500,.6);
            turnLeft( 45, .5);
            moveForwardTime(1000, -0.5);

            //Moves the servo to the scoring position for the top goal
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);

            //Outtake the block
            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            intakeServo.setPosition(intakeServoSpeed);
            sleep(2000);

             //Moves the robot back to the wall and into the depot
             moveForwardTime(2000, 0.5);
             turnRight(45,.8);
             slideLeftTime(500,.6);
             moveForwardTime(2000, 0.5);
*/

            /*
            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_GOAL_TOP);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);

            sleep(2000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OUT;
            intakeServo.setPosition(intakeServoSpeed);

            sleep(2000);

            intakeServoSpeed = StemperFiConstants.INTAKE_SERVO_SPEED_OFF;
            intakeServo.setPosition(intakeServoSpeed);

            moveForwardMM(MM_TO_TOWER / 4, .4);

            bucketServoRightPosition = StemperFiConstants.BUCKET_SERVO_RIGHT_INIT;
            bucketServoLeftPosition = 1 - bucketServoRightPosition;
            bucketServoRight.setPosition(bucketServoRightPosition);
            bucketServoLeft.setPosition(bucketServoLeftPosition);
            wormMotor.setTargetPosition(StemperFiConstants.WORM_MOTOR_INTAKE);
            wormMotor.setPower(1);
            wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (isBlue) {
                slideRightTime(1000, .6);
                turnLeft(80, .5);
            } else {
                slideLeftTime(1000, .6);
                turnRight(80, .5);
            }
            pullUpEncoders();
            moveForwardTime(1500, -.8);
*/
        }
        while (opModeIsActive()) {
            sleep(100);
        }
    }


    private void initRobot() {

        // Setup the drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftEncoder = frontLeft;
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        rightEncoder = frontRight;
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backEncoder = backLeft;
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);

        encoderServoLeft = hardwareMap.get(Servo.class, "encoderLeft");
        encoderServoRight = hardwareMap.get(Servo.class, "encoderRight");
        encoderServoCenter = hardwareMap.get(Servo.class, "encoderCenter");

        encoderServoLeft.setPosition(StemperFiConstants.ENCODER_SERVO_AUT0_LEFT);
        encoderServoRight.setPosition(StemperFiConstants.ENCODER_SERVO_AUT0_RIGHT);
        encoderServoCenter.setPosition(StemperFiConstants.ENCODER_SERVO_AUT0_CENTER);

        turntableMotor = hardwareMap.get(DcMotorEx.class, "turntable");
        turntableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        topLEDStrip = hardwareMap.get(QwiicLEDStrip.class, "TopLED");
        topLEDStrip.turnAllOff();
        topLEDStrip.setBrightness(1);

        wormMotor = hardwareMap.get(DcMotorEx.class, "worm");
        wormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormMotor.setTargetPosition(0);
        wormMotor.setPower(0.5);

        intakeServo = hardwareMap.get(Servo.class, "intake");
        intakeServo.setPosition(StemperFiConstants.INTAKE_SERVO_SPEED_OFF);

        bucketServoLeft = hardwareMap.get(Servo.class, "Left_Bucket_Servo");
        bucketServoRight = hardwareMap.get(Servo.class, "Right_Bucket_Servo");
        bucketServoLeft.setPosition(bucketServoLeftPosition);
        bucketServoRight.setPosition(bucketServoRightPosition);

        //bucketServo = hardwareMap.get(Servo.class, "bucket");
        //bucketServo.setPosition(bucketServoPosition);

        zeroTouchSensor = hardwareMap.get(TouchSensor.class, "zero");
        zeroLED = hardwareMap.get(LED.class, "zeroLed");

        do {
            greenButton.input(gamepad2.a);
            blueButton.input(gamepad2.x);
            redButton.input(gamepad2.b);
            dPadUpButton.input(gamepad2.dpad_up);
            dPadDownButton.input(gamepad2.dpad_down);
            if (redButton.onPress()) {
                isBlue = false;
            }
            if (blueButton.onPress()) {
                isBlue = true;
            }
            if (dPadUpButton.onPress()) {
                wait++;
            }
            if (dPadDownButton.onPress()) {
                wait--;
                wait = Math.max(wait, 0);
            }
            telemetry.addData("color: ", isBlue ? "Blue" : "Red");
            telemetry.addData("wait: ", wait);
            telemetry.addData("Press Green To Arm", "!");
            telemetry.update();
        } while (!greenButton.isPressed());


    }

    public void turnRight(int degrees, double power) {
        stopAndReset();
        frontRight.setPower(-power);
        backRight.setPower(-power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        long ticks = StemperFiConstants.TICKS_PER_DEGREE * degrees;
        long cp = 0;
        do {
            long rcp = rightEncoder.getCurrentPosition();
            long lcp = leftEncoder.getCurrentPosition();
            long mcp = backEncoder.getCurrentPosition();
            cp = mcp;
            telemetry.addData("tr rcp: ", rcp);
            telemetry.addData("tr lcp: ", lcp);
            telemetry.addData("tr mcp: ", mcp);
            telemetry.addData("tr target: ", ticks);
            telemetry.update();
        } while (cp < ticks);
        stopAndReset();
    }

    public void turnLeft(int degrees, double power) {
        stopAndReset();
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        long ticks = - StemperFiConstants.TICKS_PER_DEGREE * degrees;

        long cp = 0;
        do {
            long rcp =rightEncoder.getCurrentPosition();
            long lcp =leftEncoder.getCurrentPosition();
            long mcp = backEncoder.getCurrentPosition();
            cp = mcp;
            telemetry.addData("tl rcp: ", rcp);
            telemetry.addData("tl lcp: ", lcp);
            telemetry.addData("tl mcp: ", mcp);
            telemetry.addData("tl target: ", ticks);
            telemetry.update();
        } while (cp > ticks);
        stopAndReset();
    }


    public void moveForwardMM(long mm, double power) {
        long ticks = mm * StemperFiConstants.TICKS_PER_MM;
        stopAndReset();
        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }
        long cpMax = 0;
        long cpr = 0;
        long cpl = 0;
        do {
            cpr = Math.abs(rightEncoder.getCurrentPosition());
            cpl = Math.abs(leftEncoder.getCurrentPosition());
            cpMax = Math.max(cpr, cpl);
            telemetry.addData("bw   cpr: ", cpr);
            telemetry.addData("bw   cpl: ", cpl);
            telemetry.addData("bw cpMax: ", cpMax);
            telemetry.addData("bw    tp: ", ticks);
            telemetry.update();
        } while (cpMax < ticks && opModeIsActive());
        stopAndReset();
    }

    public void moveBackwardsMM(long mm, double power) {
        long ticks = mm * StemperFiConstants.TICKS_PER_MM;
        stopAndReset();;
        for (DcMotor motor : driveMotors) {
            motor.setPower(-power);
        }
        long cpMax = 0;
        long cpr = 0;
        long cpl = 0;
        do {
            cpr = Math.abs(rightEncoder.getCurrentPosition());
            cpl = Math.abs(leftEncoder.getCurrentPosition());
            cpMax = Math.max(cpr, cpl);
            telemetry.addData("bw   cpr: ", cpr);
            telemetry.addData("bw   cpl: ", cpl);
            telemetry.addData("bw cpMax: ", cpMax);
            telemetry.addData("bw    tp: ", ticks);
            telemetry.update();
        } while (cpMax < ticks && opModeIsActive());
        stopAndReset();
    }

    public void stopAndReset() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // Move the robot forwards
    public void moveForwardTime(long timems, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        }
        sleep(timems);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(0);
        }
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

    public void pullUpEncoders() {
        encoderServoLeft.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_LEFT);
        encoderServoRight.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_RIGHT);
        encoderServoCenter.setPosition(StemperFiConstants.ENCODER_SERVO_TELE_CENTER);
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