/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LaChouBase", group="FTC Lib")
public class LaChouBase extends OpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    protected MotorEx backRight, backLeft, frontRight, frontLeft;
    protected MecanumDrive mecanumDrive;
    protected Encoder leftEncoder, rightEncoder, perpEncoder;
    protected HolonomicOdometry odometry;
    protected static final double TRACK_WIDTH = 10.25;
    protected static final double CENTER_WHEEL_OFFSET = -2;
    protected static final double WHEEL_DIAMETER = 1.89;
    protected static final double TICKS_PER_REV = 2000;
    protected static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    protected GamepadEx driverOp;
    protected GamepadButton leftBumper, rightBumper, aButton, bButton, xButton, yButton;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Motors and drive
        telemetry.addLine("Initializing motors and drive");

        backRight = new MotorEx(hardwareMap, "backright", Motor.GoBILDA.RPM_312);
        backLeft = new MotorEx(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorEx(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);
        frontLeft = new MotorEx(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);

        mecanumDrive = new MecanumDrive(backRight, backLeft, frontRight, frontLeft);

        // Encoders and Odometry
        telemetry.addLine("Initializing encoders and odometry");

        leftEncoder = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        //perpEncoder.setDirection(Direction.REVERSE);

        // Reset the encoders
        leftEncoder.reset();
        rightEncoder.reset();
        perpEncoder.reset();

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACK_WIDTH,
                CENTER_WHEEL_OFFSET
        );

        // read the current position from the position tracker
        //PositionTracker.robotPose = odometry.getPose();
        odometry.updatePose(PositionTracker.robotPose);

        telemetry.addData("Robot Position at Init: ", PositionTracker.robotPose);

        // Gamepad
        telemetry.addLine("Initializing gamepads");
        driverOp = new GamepadEx(gamepad1);
        leftBumper = new GamepadButton(
                driverOp, GamepadKeys.Button.LEFT_BUMPER);
        rightBumper = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        xButton = new GamepadButton(driverOp, GamepadKeys.Button.X);
        yButton = new GamepadButton(driverOp, GamepadKeys.Button.Y);
        aButton = new GamepadButton(driverOp, GamepadKeys.Button.A);
        bButton = new GamepadButton(driverOp, GamepadKeys.Button.B);

        // Tell the driver that initialization is complete.
        telemetry.addLine("Initialization complete");
        telemetry.addLine("Press Start on the Driver Station");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        showGamepadTelemetry();

        odometry.updatePose();
        PositionTracker.robotPose = odometry.getPose();
        showOdometryTelemetry();

        mecanumDrive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                -PositionTracker.robotPose.getHeading() * 57.2958,
                true
        );

/*
        mecanum.driveRobotCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                true
        );
*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void showGamepadTelemetry() {
        telemetry.addLine("Gamepad");
        telemetry.addData("Left X", driverOp.getLeftX());
        telemetry.addData("Left Y", driverOp.getLeftY());
        telemetry.addData("Right X", driverOp.getRightX());
        telemetry.addData("Right Y", driverOp.getRightY());
    }

    private void showOdometryTelemetry() {
        telemetry.addLine("Odometry");
        telemetry.addData("Heading", PositionTracker.robotPose.getHeading());
        telemetry.addData("Heading (Degrees)", PositionTracker.robotPose.getHeading() * 57.2958);
        telemetry.addData("Pose", PositionTracker.robotPose);
        telemetry.addData("Left Encoder", leftEncoder.getDistance());
        telemetry.addData("Right Encoder", rightEncoder.getDistance());
        telemetry.addData("Perp Encoder", perpEncoder.getDistance());
    }


}
