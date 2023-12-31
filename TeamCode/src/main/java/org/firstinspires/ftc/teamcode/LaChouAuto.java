package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.vision.UGBasicHighGoalPipeline;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
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

@Autonomous(name="LaChouAuto", group="Robot")
public class LaChouAuto extends LinearOpMode {

    private MotorEx backRight, backLeft, frontRight, frontLeft;
    private MecanumDrive mecanum;
    private ColorSensor colorSensor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private TfodProcessor tfodProcessor;

    private LaChouAuto.ContourPipeline pipeline;
    private OpenCvWebcam webcam;

    private Motor.Encoder leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;
    public static final double TRACK_WIDTH = 10.25;
    public static final double CENTER_WHEEL_OFFSET = -2;
    public static final double WHEEL_DIAMETER = 1.89;
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;



    @Override
    public void runOpMode() {

        // Motors and drive
        telemetry.addLine("Initializing motors and drive");

        backRight = new MotorEx(hardwareMap, "backright", Motor.GoBILDA.RPM_312);
        backLeft = new MotorEx(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorEx(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);
        frontLeft = new MotorEx(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);

        mecanum = new MecanumDrive(backRight, backLeft, frontRight, frontLeft);

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

        // Color sensor
        telemetry.addLine("Initializing color sensor");
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

        // TensorFlow processor
        telemetry.addLine("Initializing tensorflow processor");
        tfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .build();

        // Vision Portal
        telemetry.addLine("Initializing vision portal");
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .addProcessor(tfodProcessor)
                .build();

        // Camera
        telemetry.addLine("Initializing webcam");
        /*
         * Use a camera monitor
        int cameraMonitorViewId = hardwareMap
                .appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
         */
        // Don't use a camera monitor
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Pipeline
        telemetry.addLine("Initializing pipeline");
        pipeline = new LaChouAuto.ContourPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Tell the driver that initialization is complete.
        telemetry.addLine("Initialization complete");

        telemetry.update();

        waitForStart();

/*
        mecanum.driveRobotCentric(.5,.5,.5);
        sleep(500);
        mecanum.driveRobotCentric(0,0,0);
        sleep(500);
        mecanum.driveRobotCentric(-.5,-.5,-.5);
        sleep(500);
*/

        // Detect the Spike Mark that the Team Prop is on - 1, 2, or 3
        int spikeMark = detectSpikeMark();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Calls", pipeline.getCalls());
            telemetry.addData("Contours Found", pipeline.getContours().size());
            telemetry.addData("Potential Pixels Found", pipeline.getPixels().size());
            telemetry.update();
        }

        // Push pre-loaded purple pixel to the proper Spike Mark (20 points)

        // Detect the Backdrop target location based on spike mark (1,2,3) and april tag

        // Yellow pixel is preloaded in grabber
        // Place the pixel on the backdrop (20 points)

        // Go to the white pixel storage area w/o disturbing the purple pixel

        // Intake a white pixel

        // Go to the Backdrop

        // Place the white pixel on the backdrop (3 or 5 points)

    }

    private int detectSpikeMark() {
        return 0;
    }

    class ContourPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        int calls = 0;
        List<MatOfPoint> contours = new ArrayList();
        List<MatOfPoint> pixels = new ArrayList();

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            calls++;

            // Convert to HSV color space
            Mat hsvImage = new Mat();
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

            // Define the range of the specific color to detect
            // Hue range: 0-180
            // Saturation range: 0-255
            // Value range: 0-255
            // Scalar lowerBound = new Scalar(lowerHue, lowerSaturation, lowerValue);
            // Scalar upperBound = new Scalar(upperHue, upperSaturation, upperValue);

            // Red
            // Sample HSV Values:
            // 348, 100, 79
            // 354, 58, 96
            // 353, 75, 82
            // 354, 84, 80
            Scalar lowerBound = new Scalar(160, 0, 0);
            Scalar upperBound = new Scalar(180, 255, 255);

            // Blue
            // Sample HSV Values:
            // 223, 100, 98
            // 224, 100, 99
            // 213, 80, 100
            // 212, 68, 100
//            Scalar lowerBound = new Scalar(lowerHue, lowerSaturation, lowerValue);
//            Scalar upperBound = new Scalar(upperHue, upperSaturation, upperValue);

            // Create a mask for the specific color
            Mat colorMask = new Mat();
            Core.inRange(hsvImage, lowerBound, upperBound, colorMask);

            // Find contours
            contours.clear();
            Mat hierarchy = new Mat();
            Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Filter through contours to find likely pixels
            pixels.clear();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 10 && area < 100) {
                    pixels.add(contour);
                }
            }

            return colorMask;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused) {
                webcam.pauseViewport();
            }
            else {
                webcam.resumeViewport();
            }
        }

        public int getCalls() {
            return calls;
        }

        public List getContours() {
            return contours;
        }

        public List getPixels() {
            return pixels;
        }

    }
}