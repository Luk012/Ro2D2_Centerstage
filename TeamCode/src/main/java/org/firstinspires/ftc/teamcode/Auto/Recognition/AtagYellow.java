package org.firstinspires.ftc.teamcode.Auto.Recognition;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;


import java.util.List;

public class AtagYellow implements VisionProcessor {

    public enum Location{
        LEFT,
        RIGHT,
        MIDDLE,
        NONE,
    }

    private AprilTagProcessor aprilTagProcessor;
    public int desiredTag;
    public boolean funny = false;


    public Location location = null;

    public int  leftZoneAverage, rightZoneAverage;

    public AtagYellow(AprilTagProcessor aprilTagProcessor, int desiredTag) {
        this.aprilTagProcessor = aprilTagProcessor;
        this.desiredTag = desiredTag;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        aprilTagProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to grayscale
        Mat grayFrame = new Mat();
        Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_RGB2GRAY);

        List<AprilTagDetection> detectionList = aprilTagProcessor.getDetections();
       // Rect test = new Rect(50, 50, 50, 50);
        //Imgproc.rectangle(frame, test, new Scalar(0, 255, 0), 7);
        try {
            if (detectionList != null) {
                for (AprilTagDetection detection : detectionList) {
                    if (detection.id == desiredTag) {

                    funny = true;

                        int leftX = Integer.MAX_VALUE;
                        int rightX = Integer.MIN_VALUE;
                        int topY = Integer.MIN_VALUE;
                        int bottomY = Integer.MAX_VALUE;

                        for (Point point : detection.corners) {
                            if (point.x < leftX) leftX = (int) point.x;
                            if (point.x > rightX) rightX = (int) point.x;
                            if (point.y > topY) topY = (int) point.y;
                            if (point.y < bottomY) bottomY = (int) point.y;
                        }

                        int tagCenterX = (int) detection.center.x;
                        int tagCenterY = (int) detection.center.y;

                        int tagWidth = rightX - leftX;
                        int tagHeight = topY - bottomY;

                        int inclusionZoneWidth = (int) (tagWidth * 1.5);
                        int inclusionZoneHeight = (int) (tagHeight * 1.5);

                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, (int) (tagCenterY - tagHeight * 2.5), inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, (int) (tagCenterY - tagHeight * 2.5), inclusionZoneWidth, inclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 5);

                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 5);

                         leftZoneAverage = meanIntensity(grayFrame, leftInclusionZone);
                         rightZoneAverage = meanIntensity(grayFrame, rightInclusionZone);

                        if (leftZoneAverage <= 25 && rightZoneAverage <= 25) {
                            location = Location.NONE;
                        } else if(leftZoneAverage > 25 && rightZoneAverage > 25){
                            location = Location.MIDDLE;
                        } else if(leftZoneAverage >= 50 && rightZoneAverage <= 25)
                        {
                            location = Location.LEFT;
                        } else
                        {
                            location  = Location.RIGHT;
                        }
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Implement if needed
    }

    public int meanIntensity(Mat frame, Rect inclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
                    continue;
                }

                double[] data = frame.get(y, x);
                if (data != null && data.length > 0) {
                    sum += data[0];
                    count++;
                }
            }
        }

        return count > 0 ? sum / count : 0;
    }
}
