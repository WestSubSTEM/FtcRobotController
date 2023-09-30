import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Learn 1", group = "programming")
public class TeleLearnProgramming extends OpMode {

    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor joe;

    @Override
    public void init() {
        frontLeft =hardwareMap.get(DcMotor.class,"driveFrontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight=hardwareMap.get(DcMotor.class,"driveBackRight");
        frontRight=hardwareMap.get(DcMotor.class,"driveFrontRight");
        backLeft =hardwareMap.get(DcMotor.class,"driveBackLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        joe=hardwareMap.get(DcMotor.class,"dave");
        joe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joe.setTargetPosition(0);
        joe.setPower(0);
        joe.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        //drive the robot
        /*ll
        llll

        frontLeft.setPower(-gamepad1.left_stick_y);
        backLeft.setPower(-gamepad1.right_stick_y);
        frontRight.setPower(-gamepad2.left_stick_y);
        backRight.setPower(-gamepad2.right_stick_y);

         */
       if(gamepad1.a){
           joe.setTargetPosition(1000);
           joe.setPower(.5);
       }else if (gamepad1.b){
           joe.setTargetPosition(-1000);
           joe.setPower(1);
       }else if (gamepad1.x){
           joe.setTargetPosition(0);
           joe.setPower(.2);
       }
        telemetry.addData("power",joe.getPower());
        telemetry.addData("position",joe.getCurrentPosition());
        telemetry.update();
    }


    @Override
    public void stop() {
    }
}

