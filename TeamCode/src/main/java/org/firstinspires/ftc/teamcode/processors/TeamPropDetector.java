package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class TeamPropDetector implements VisionProcessor {

    int calls = 0;

    List<MatOfPoint> contours = new ArrayList();
    List<MatOfPoint> pixels = new ArrayList();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        calls++;

        // Convert to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

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
            if (area > 200) {
                pixels.add(contour);
            }
        }

        return colorMask;
    }

    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {

    }

    public boolean foundPixels() {
        return pixels.size() > 0;
    }

    public int getSpikeMarkZone() {
        return 0;
    }

    public int getCalls() {
        return calls;
    }

    public int getNumberOfPixels() {
        return pixels.size();
    }
}
