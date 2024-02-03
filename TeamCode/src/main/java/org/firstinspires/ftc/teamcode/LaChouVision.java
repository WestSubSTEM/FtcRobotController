package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.STEMperFiConstants.TeamPropColor;

@Autonomous(name="LaChouVision", group="Robot")
public class LaChouVision extends LaChouBase {

    private ColorSensor colorSensor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TeamPropDetector tpdProcessor;

    int state;
    int regionGuess;
    TeamPropColor colorGuess;

    @Override
    public void init() {
        super.init();

        // Color sensor
        telemetry.addLine("Initializing color sensor");
        telemetry.update();
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // AprilTag processor
        telemetry.addLine("Initializing april tag processor");
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // TeamPropDetector processor
        telemetry.addLine("Initializing TeamPropDetector processor");
        tpdProcessor = new TeamPropDetector();

        // Vision Portal
        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        VisionPortal.Builder vpBuilder = new VisionPortal.Builder();
        vpBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpBuilder.addProcessor(tpdProcessor);
        vpBuilder.setCameraResolution(new Size(640, 480));
        visionPortal = vpBuilder.build();

    }

    @Override
    public void start() {
        state = 0;
        colorGuess = TeamPropColor.UNKNOWN;
        regionGuess = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        switch (state) {

            // Detect the location of the team prop
            case 0:
                telemetry.addLine("Guessing");
                telemetry.addData("Number of Calls", tpdProcessor.getNumberCalls());
                telemetry.addData("Number of Finds", tpdProcessor.getNumberFinds());
                telemetry.addData("Number of Reds", tpdProcessor.getNumberReds());
                telemetry.addData("Number of Blues", tpdProcessor.getNumberBlues());
                telemetry.addData("Comment", tpdProcessor.getComment());
                if (tpdProcessor.isGuessed()) {
                    this.colorGuess = tpdProcessor.getColorGuess();
                    this.regionGuess = tpdProcessor.getRegionGuess();
                    tpdProcessor.disable();
                    state = 1;
                }
                sleep(200); // Polite pause allowing for interrupts
                break;

            // Push pre-loaded purple pixel to the proper Spike Mark (20 points)'
            case 1:
                telemetry.addData("Comment", tpdProcessor.getComment());
                if (this.colorGuess == TeamPropColor.BLUE) {
                    telemetry.addData("Color", "Blue");
                } else if (this.colorGuess == TeamPropColor.RED) {
                    telemetry.addData("Color", "Red");
                } else {
                    telemetry.addData("Color", "Unknown");
                }
                telemetry.addData("Region", this.regionGuess);
                state = 1;
                sleep(500); // Polite pause allowing for interrupts
                //moveToSpikeMark(this.colorGuess, this.regionGuess);
                break;

            // Move to the correct backdrop
            case 2:
                break;

            // Place the pre-loaded yellow pixel on the backdrop (20 points)
            case 3:
                break;

            // Go to the white pixel storage area w/o disturbing the purple pixel
            case 4:
                break;

            // Intake a white pixel
            case 5:
                break;

            // Go to the Backdrop
            case 6:
                break;

            // Place the white pixel on the backdrop (3 or 5 points)
            case 7:
                break;

            default:
                telemetry.addLine("Auto complete");

        }
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void moveToSpikeMark(int colorGuess, int regionGuess) {
        switch (regionGuess) {
            case 1:
                // Move to the left

                // Move forward
                mecanumDrive.driveRobotCentric(0, .2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() > -31);

                // Turn left
                mecanumDrive.driveRobotCentric(0, 0, -.2, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("Rotation", odometry.getPose().getRotation());
                } while (odometry.getPose().getRotation().getDegrees() > -90);

                // Move forward
                mecanumDrive.driveRobotCentric(0, .2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() > -31);

                // Stop
                mecanumDrive.driveRobotCentric(0, 0, 0, false);
                sleep(2_000);

                // Move backward
                mecanumDrive.driveRobotCentric(0, -0.2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() < -21);
                mecanumDrive.driveRobotCentric(0, 0, 0, false);

                sleep(2_000);
                break;
            case 2:
                // Move to the center
                // Move to the right
                mecanumDrive.driveRobotCentric(0, .2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() > -31);
                mecanumDrive.driveRobotCentric(0, 0, 0, false);
                sleep(2_000);
                mecanumDrive.driveRobotCentric(0, -0.2, 0, false);
                do {
                    odometry.updatePose();
                    telemetry.addData("x", odometry.getPose().getX());
                    telemetry.addData("y", odometry.getPose().getY());
                    telemetry.update();
                } while (odometry.getPose().getX() < -21);
                mecanumDrive.driveRobotCentric(0, 0, 0, false);
                sleep(2_000);
                break;
            case 3:
                // Move to the right
                break;
            default:
                telemetry.addLine("Invalid spike mark zone");
        }
    }

}