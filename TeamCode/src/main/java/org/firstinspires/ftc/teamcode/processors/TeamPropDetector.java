package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
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

    boolean enabled = true;

    public Rect region1Rect;
    public Rect region2Rect = new Rect(20, 20, 50, 50);
    public Rect region3Rect = new Rect(20, 20, 50, 50);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    private static double findLargestContour(Mat image, Scalar lowerBound, Scalar upperBound) {
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

        this.region1Rect = new Rect(0 , height / 3 * 2, width / 3, height / 3);
        this.region2Rect = new Rect(width/3, height / 3 * 2, width / 3, height / 3);
        this.region3Rect = new Rect(width/3 * 2, height / 3 * 2, width / 3, height / 3);

        List<Mat> regions = new ArrayList<>();
        regions.add(hsvImage.submat(height / 3 * 2, height, 0, width / 3));
        regions.add(hsvImage.submat(height / 3 * 2, height, width / 3, (width / 3) * 2));
        regions.add(hsvImage.submat(height / 3 * 2, height, (width / 3) * 2, width));

        Scalar[][] colorBounds = {
                { new Scalar(90, 50, 50), new Scalar(100, 255, 255) },   // Blue
                { new Scalar(0, 50, 50), new Scalar(10, 255, 255) },    // Red 1
                { new Scalar(160, 50, 50), new Scalar(180, 255, 255) }  // Red 2
        };

        double largestArea = 0;

        for (int regionIndex = 0; regionIndex < regions.size(); regionIndex++) {
            for (int colorIndex = 0; colorIndex < colorBounds.length; colorIndex++) {
                double area = findLargestContour(regions.get(regionIndex), colorBounds[colorIndex][0], colorBounds[colorIndex][1]);
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

        if (this.enabled) {
            guessProp(frame);
        }

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        if (this.region1Rect != null) {
            canvas.drawRect(makeGraphicsRect(this.region1Rect, scaleBmpPxToCanvasPx), rectPaint);
        }
        if (this.region2Rect != null) {
            canvas.drawRect(makeGraphicsRect(this.region2Rect, scaleBmpPxToCanvasPx), rectPaint);
        }
        if (this.region3Rect != null) {
            canvas.drawRect(makeGraphicsRect(this.region3Rect, scaleBmpPxToCanvasPx), rectPaint);
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public boolean isGuessed() {
        return this.guessed;
    }

    public int getColorGuess() {
        return this.colorGuess;
    }

    public int getRegionGuess() {
        return this.regionGuess;
    }

    public void enable() {
        this.enabled = true;
    }
    public void disable() {
        this.enabled = false;
    }

    public int getNumberOfCalls() {
        return this.calls;
    }
}
