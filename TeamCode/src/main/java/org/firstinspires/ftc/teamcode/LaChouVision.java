package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.TeamPropDetectorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="LaChouVision", group="Robot")
public class LaChouVision extends LaChouBase {

    private ColorSensor colorSensor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TeamPropDetectorProcessor tpdProcessor;

    int state;
    int regionGuess;
    int colorGuess;

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
        tpdProcessor = new TeamPropDetectorProcessor();

        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor,
                tpdProcessor);
    }


    @Override
    public void start() {
        state = 0;
        colorGuess = 0;
        regionGuess = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("State", state);
        switch (state) {

            // Detect the location of the team prop
            case 0:
                telemetry.addLine("Guessing");
                while(!tpdProcessor.guessed()) {
                    this.colorGuess = tpdProcessor.getColorGuess();
                    this.regionGuess = tpdProcessor.getRegionGuess();
                }
                //tpdProcessor.stop();
                telemetry.addData("Color", this.colorGuess);
                telemetry.addData("Region", this.regionGuess);
                state = 1;
                break;

            // Push pre-loaded purple pixel to the proper Spike Mark (20 points)'
            case 1:
                moveToSpikeMark(this.colorGuess, this.regionGuess);
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


    private void showColorSensorTelemetry() {
        telemetry.addLine("Color Sensor");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
    }

    private void moveToSpikeMark(int colorGuess, int regionGuess) {
        switch (regionGuess) {
            case 1:
                // Move to the left
                break;
            case 2:
                // Move to the middle
                break;
            case 3:
                // Move to the right
                break;
            default:
                telemetry.addLine("Invalid spike mark zone");
        }
    }

}