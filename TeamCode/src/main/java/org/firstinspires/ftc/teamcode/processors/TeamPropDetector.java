package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

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

    int numberCalls = 0;

    String comment;

    // 0 = no guess, 1 = blue, 2 = red
    TeamPropColor colorGuess = TeamPropColor.UNKNOWN;

    // 0 = no guess, 1 = region 1, 2 = region 2, 3 = region 3
    int regionGuess = 0;

    boolean guessed = false;

    boolean enabled = true;

    public Rect region1Rect;
    public Rect region2Rect;
    public Rect region3Rect;

    public List<ContourInfo> foundContours;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

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

        return largestContour;
    }

    public Mat guessProp(Mat image) {

        // Convert the image to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define color bounds of interest (the blue and red team props)
        Scalar[][] colorBounds = {
                {new Scalar(90, 50, 50), new Scalar(130, 255, 255)},   // Blue
                {new Scalar(0, 50, 50), new Scalar(10, 255, 255)},    // Red 1
                {new Scalar(160, 50, 50), new Scalar(180, 255, 255)}  // Red 2
        };

        // Define the regions of interest
        int imageHeight = hsvImage.rows();
        int imageWidth = hsvImage.cols();

        int regionHeight = imageHeight / 3;
        int regionWidth = imageWidth / 3;

        int regionStartHeight = imageHeight / 3 * 2;

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
                    double area = Imgproc.contourArea(largestContour);

                    // Offset the contour coordinates to match the original image's coordinate system
                    MatOfPoint offsetContour = new MatOfPoint();
                    offsetContour.create((int) largestContour.size().height, 1, CvType.CV_32SC2);
                    for (int i = 0; i < largestContour.rows(); i++) {
                        double[] point = largestContour.get(i, 0);
                        point[0] = point[0] + (regionIndex * regionWidth); // Adjust x-coordinate
                        point[1] = point[1] + regionStartHeight; // Adjust y-coordinate
                        offsetContour.put(i, 0, point);
                    }

                    // Create and store the contour information
                    TeamPropColor teamPropColor = TeamPropColor.UNKNOWN;
                    if (colorIndex == 0) {
                        teamPropColor = TeamPropColor.BLUE;
                    } else {
                        teamPropColor = TeamPropColor.RED;
                    }
                    foundContours.add(new ContourInfo(offsetContour, area, teamPropColor, regionIndex + 1));
                } else {
                    comment = "No contour found in " + regionIndex + " for color " + colorIndex;
                }
            }
        }

        // Find the largest contour of all the found contours
        double largestArea = 0;
        for (ContourInfo contourInfo : foundContours) {
            if (contourInfo.area > largestArea) {
                largestArea = contourInfo.area;
                this.colorGuess = contourInfo.color;
                this.regionGuess = contourInfo.region;
            }
        }

        if (this.colorGuess != TeamPropColor.UNKNOWN && this.regionGuess != 0) {
            this.guessed = true;
        }

        return hsvImage;
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        numberCalls++;

        Mat hsvImage = frame;

        if (this.enabled) {
            // Reset data
            this.comment = "";
            this.colorGuess = TeamPropColor.UNKNOWN;
            this.regionGuess = 0;
            this.foundContours = new ArrayList<>();
            this.guessed = false;

            hsvImage = guessProp(frame);
        }

        return hsvImage;
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
        rectPaint.setColor(Color.WHITE);
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

        if (this.foundContours != null) {
            // Draw the found contours
            Paint redPaint = new Paint();
            redPaint.setColor(Color.RED);
            redPaint.setStyle(Paint.Style.STROKE);
            redPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint bluePaint = new Paint();
            bluePaint.setColor(Color.BLUE);
            bluePaint.setStyle(Paint.Style.STROKE);
            bluePaint.setStrokeWidth(scaleCanvasDensity * 4);

            for (ContourInfo contourInfo : foundContours) {
                if (contourInfo.color == TeamPropColor.BLUE) {
                    canvas.drawRect(makeGraphicsRect(Imgproc.boundingRect(contourInfo.contour), scaleBmpPxToCanvasPx), bluePaint);
                } else {
                    canvas.drawRect(makeGraphicsRect(Imgproc.boundingRect(contourInfo.contour), scaleBmpPxToCanvasPx), redPaint);
                }
            }
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

    public TeamPropColor getColorGuess() {
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

    public int getNumberCalls() {
        return this.numberCalls;
    }

    public int getNumberFinds() {
        return this.foundContours.size();
    }

    public int getNumberReds() {
        if (this.foundContours == null) {
            return 0;
        } else {
            int count = 0;
            for (ContourInfo contourInfo : foundContours) {
                if (contourInfo.color == TeamPropColor.RED) {
                    count++;
                }
            }
            return count;
        }
    }

    public int getNumberBlues() {
        if (this.foundContours == null) {
            return 0;
        } else {
            int count = 0;
            for (ContourInfo contourInfo : foundContours) {
                if (contourInfo.color == TeamPropColor.BLUE) {
                    count++;
                }
            }
            return count;
        }
    }

    public String getComment() {
        return this.comment;
    }
}
