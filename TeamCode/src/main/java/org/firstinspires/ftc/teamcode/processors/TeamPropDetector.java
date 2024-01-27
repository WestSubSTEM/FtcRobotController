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

    List<MatOfPoint> contours = new ArrayList<>();
    List<MatOfPoint> pixels = new ArrayList<>();

    // 0 = no guess, 1 = blue, 2 = red
    int colorGuess = 0;

    // 0 = no guess, 1 = region 1, 2 = region 2, 3 = region 3
    int regionGuess = 0;

    boolean guessed = false;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    public static double findLargestContour(Mat image, Scalar lowerBound, Scalar upperBound) {
        Mat mask = new Mat();
        Core.inRange(image, lowerBound, upperBound, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double largestArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestArea) {
                largestArea = area;
            }
        }

        return largestArea;
    }

    public void guessProp(Mat image) {
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        int height = hsvImage.rows();
        int width = hsvImage.cols();

        List<Mat> regions = new ArrayList<>();
        regions.add(hsvImage.submat(height / 3 * 2, height, 0, width / 3));
        regions.add(hsvImage.submat(height / 3 * 2, height, width / 3, (width / 3) * 2));
        regions.add(hsvImage.submat(height / 3 * 2, height, (width / 3) * 2, width));

        Scalar[] colorBounds = {
                new Scalar(90, 50, 50),   // Blue
                new Scalar(0, 50, 50),    // Red 1
                new Scalar(160, 50, 50)  // Red 2
        };

        double largestArea = 0;

        for (int regionIndex = 0; regionIndex < regions.size(); regionIndex++) {
            for (int colorIndex = 0; colorIndex < colorBounds.length; colorIndex++) {
                double area = findLargestContour(regions.get(regionIndex), colorBounds[colorIndex], colorBounds[colorIndex]);
                if (area > largestArea) {
                    largestArea = area;
                    if (colorIndex == 0) {
                        this.colorGuess = 1;
                    } else {
                        this.colorGuess = 2;
                    }
                    this.regionGuess = regionIndex + 1;
                }
            }
        }

        this.guessed = true;
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        calls++;

        guessProp(frame);

        return frame;
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
        return 1;
    }

    public boolean guessed() {
        return this.guessed;
    }

    public int getColorGuess() {
        return this.colorGuess;
    }

    public int getRegionGuess() {
        return this.regionGuess;
    }
}
