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
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="LaChouBot", group="FTC Lib")
public class LaChouBot extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // input motors exactly as shown below
    MecanumDrive mecanum;
    GamepadEx driverOp;
    GamepadButton leftBumper, rightBumper;
    RevIMU imu;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        imu = new RevIMU(hardwareMap);
        imu.init();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        //imu.initialize(new IMU.Parameters(orientationOnRobot));


        // the extended gamepad object
        driverOp = new GamepadEx(gamepad1);

        leftBumper = new GamepadButton(
                driverOp, GamepadKeys.Button.LEFT_BUMPER);
        rightBumper = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);

        Motor frontLeft = new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312);

       // mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        mecanum = new MecanumDrive(backRight, backLeft , frontRight, frontLeft);
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

        double lx = driverOp.getLeftX();
        double ly = driverOp.getLeftY();
        double rx = driverOp.getRightX();
        double degrees = imu.getRotation2d().getDegrees();

        if (!leftBumper.get() && !rightBumper.get()) {
            telemetry.addLine("Robot Centric");
            mecanum.driveRobotCentric(
                    lx,
                    ly,
                    rx,
                    false
            );
        } else {
            telemetry.addLine("Field Centric");
            mecanum.driveFieldCentric(
                    lx,
                    ly,
                    rx,
                    degrees,   // gyro value passed in here must be in degrees
                    false
            );
        }
        telemetry.addData("degrees", degrees);
        telemetry.addData("Left X", lx);
        telemetry.addData("Left Y", ly);
        telemetry.addData("Right X", rx);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
