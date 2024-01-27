package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="LaChouTfod", group="Robot")
public class LaChouTfod extends LaChouBase {

    private ColorSensor colorSensor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TfodProcessor tfodProcessor;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20240109_165853.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20240106_205617.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "red prop",
            "blue prop"
    };

    int state;
    int spikeMarkZone;

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
                // == CAMERA CALIBRATION ==
                // This is for the wide angle lens webcam
                // Comment out this line if using the Logitech webcam
                .setLensIntrinsics(400.147, 400.147, 311.482, 251.294)
                // ... these parameters are fx, fy, cx, cy.
                .build();


        // TensorFlow processor
        telemetry.addLine("Initializing tensorflow processor");
        tfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTagProcessor,
                tfodProcessor);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        state = 0;
        spikeMarkZone = 0;
    }

    @Override
    public void loop() {

            telemetryTfod();

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();
        telemetry.addData("No. Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


}