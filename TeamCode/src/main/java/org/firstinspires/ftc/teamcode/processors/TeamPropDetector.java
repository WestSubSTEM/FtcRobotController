package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.STEMperFiConstants.TeamPropColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;


class ContourInfo {
    MatOfPoint contour;
    double area;
    TeamPropColor color;
    int region; // 1, 2, or 3

    ContourInfo(MatOfPoint contour, double area, TeamPropColor color, int region) {
        this.contour = contour;
        this.area = area;
        this.color = color;
        this.region = region;
    }
}

public class TeamPropDetector implements VisionProcessor {

    private static final String TAG = "TeamPropDetector";

    TeamPropColor colorGuess = TeamPropColor.UNKNOWN;

    // 0 = no guess, 1 = region 1, 2 = region 2, 3 = region 3
    int regionGuess = 0;

    boolean guessed = false;

    boolean enabled = true;

    // Set to false if the detector is not ready to process new frames
    boolean ready = true;

    Rect region1Rect;
    Rect region2Rect;
    Rect region3Rect;

    int regionWidth;
    int regionHeight;
    int regionStartHeight;

//    List<ContourInfo> foundContours;
    CopyOnWriteArrayList<ContourInfo> foundContours;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //Log.d(TAG, "Initializing TeamPropDetector");
        this.reset();
    }

    private MatOfPoint findLargestContour(Mat image, Scalar lowerBound, Scalar upperBound) {
        Mat mask = new Mat();
        Core.inRange(image, lowerBound, upperBound, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double largestArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestArea) {
                largestArea = area;
                largestContour = contour; // Keep track of the largest contour
            }
        }

        mask = null;
        contours = null;
        hierarchy = null;

        if (largestContour != null) {
            // Check the ratio of the largest contour
            Rect rect = Imgproc.boundingRect(largestContour);
            double aspect = Math.max(rect.height / rect.width, rect.width / rect.height);
            if (aspect < 1.15) {
                // If the ratio is reasonably box-ish (i.e. not a spike mark)
                return largestContour;
            } else {
                // Likely a spike mark
                return null;
            }
        } else {
            return null;
        }
    }

    public void guessProp(Mat image) {

        // Convert the image to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Define color bounds of interest (the blue and red team props)
        Scalar[][] colorBounds = {
                {new Scalar(90, 50, 50), new Scalar(130, 255, 255)},   // Blue
                {new Scalar(0, 50, 50), new Scalar(10, 255, 255)},    // Red 1
                {new Scalar(160, 50, 50), new Scalar(180, 255, 255)}  // Red 2
        };

        // Define the regions of interest
        int imageHeight = hsvImage.rows();
        int imageWidth = hsvImage.cols();

        regionHeight = imageHeight / 3;
        regionWidth = imageWidth / 3;
        regionStartHeight = regionHeight * 2;

        region1Rect = new Rect(0, regionStartHeight, regionWidth, regionHeight);
        region2Rect = new Rect(imageWidth / 3, regionStartHeight, regionWidth, regionHeight);
        region3Rect = new Rect(imageWidth / 3 * 2, regionStartHeight, regionWidth, regionHeight);

        // Create the regions of interest
        List<Mat> regions = new ArrayList<>();
        regions.add(hsvImage.submat(regionStartHeight, imageHeight, region1Rect.x, region1Rect.x + region1Rect.width));
        regions.add(hsvImage.submat(regionStartHeight, imageHeight, region2Rect.x, region2Rect.x + region2Rect.width));
        regions.add(hsvImage.submat(regionStartHeight, imageHeight, region3Rect.x, region3Rect.x + region3Rect.width));

        // Find the largest contour in each region
        for (int regionIndex = 0; regionIndex < regions.size(); regionIndex++) {
            for (int colorIndex = 0; colorIndex < colorBounds.length; colorIndex++) {
                MatOfPoint largestContour = findLargestContour(regions.get(regionIndex), colorBounds[colorIndex][0], colorBounds[colorIndex][1]);
                if (largestContour != null) {

                    // Create and store the contour information
                    TeamPropColor teamPropColor = TeamPropColor.UNKNOWN;
                    if (colorIndex == 0) {
                        teamPropColor = TeamPropColor.BLUE;
                    } else {
                        teamPropColor = TeamPropColor.RED;
                    }
                    foundContours.add(
                            new ContourInfo(
                                    largestContour,
                                    Imgproc.contourArea(largestContour),
                                    teamPropColor,
                                    regionIndex + 1));
                }
            }
        }

        // Find the largest contour of all the found contours
        ContourInfo largestContourInfo = null;
        for (ContourInfo contourInfo : foundContours) {
            if (largestContourInfo == null || contourInfo.area > largestContourInfo.area) {
                largestContourInfo = contourInfo;
            }
        }

        if (largestContourInfo != null) {
            //Log.d(TAG, "Largest contour: area " + largestContourInfo.area + " | color " + largestContourInfo.color + " region " + largestContourInfo.region);
            this.colorGuess = largestContourInfo.color;
            this.regionGuess = largestContourInfo.region;
            this.guessed = true;
        }

        hsvImage = null;
    }

    public void reset() {
//        Log.d(TAG, "Resetting TeamPropDetector");
        this.colorGuess = TeamPropColor.UNKNOWN;
        this.regionGuess = 0;
//        this.foundContours = new ArrayList<>();
        this.foundContours = new CopyOnWriteArrayList<>();
        this.guessed = false;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (this.enabled && this.ready) {
            // Set to block to prevent clobbering internal data, esp. while drawing
            this.ready = false;
            // Reset data
            this.reset();
            guessProp(frame);
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {

        // Draw the regions of interest
        Paint rectPaint = new Paint();
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        rectPaint.setColor(Color.WHITE);

        if (this.region1Rect != null) {
            canvas.drawRect(makeGraphicsRect(this.region1Rect, scaleBmpPxToCanvasPx), rectPaint);
        }
        if (this.region2Rect != null) {
            canvas.drawRect(makeGraphicsRect(this.region2Rect, scaleBmpPxToCanvasPx), rectPaint);
        }
        if (this.region3Rect != null) {
            canvas.drawRect(makeGraphicsRect(this.region3Rect, scaleBmpPxToCanvasPx), rectPaint);
        }

        if (this.foundContours != null) {

            for (ContourInfo contourInfo : foundContours) {
                int xOffset = (contourInfo.region - 1) * regionWidth;
                int yOffset = regionStartHeight;

                // Get the bounding box
                MatOfPoint contour = new MatOfPoint();
                contourInfo.contour.convertTo(contour, CvType.CV_32S);
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Offset the rect based on the region
                boundingRect.x += xOffset;
                boundingRect.y += yOffset;

                // Set paint color based on the team color
                if (contourInfo.color == TeamPropColor.BLUE) {
                    rectPaint.setColor(Color.BLUE);
                } else if (contourInfo.color == TeamPropColor.RED) {
                    rectPaint.setColor(Color.RED);
                }

                canvas.drawRect(makeGraphicsRect(boundingRect, scaleBmpPxToCanvasPx), rectPaint);
            }
        }

        this.ready = true;
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

    public TeamPropColor getColorGuess() {
        return this.colorGuess;
    }

    public int getRegionGuess() {
        return this.regionGuess;
    }

    public void enable() {
        //Log.d(TAG, "Enabling TeamPropDetector");
        this.enabled = true;
    }

    public void disable() {
        //Log.d(TAG, "Disabling TeamPropDetector");
        this.enabled = false;
    }

}
