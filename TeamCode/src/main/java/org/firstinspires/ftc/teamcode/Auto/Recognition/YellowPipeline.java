package org.firstinspires.ftc.teamcode.Auto.Recognition;

import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.location.center;
import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.location.left;
import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.location.none;
import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.location.right;

import android.graphics.Canvas;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class YellowPipeline implements VisionProcessor {

    public enum location{
        left,
        center,
        right,
        none,
        nothing,
    }

    public enum Side{
        blue,
        red,
        nothing,
    }

    public int targetAprilTagID = 5;
    public int yellowTresholdLeftSide = 120; // treshold when the robot is in the left side / in front of the AprilTag
    public int yellowTresholdRightSide = 85; // treshold when the robot is in the right side of the AprilTag; decreases to the right
    public int yellowTresholdCenter = 80; // treshold when the pixel is on top of the AprilTag (center case)

    public location preloadedZone = center; //default
    boolean diditsee = false;
   public Side side = Side.nothing;
    public int clawpoz = 0;


    private AprilTagProcessor aprilTag;

    public Scalar rectColor = new Scalar(0, 255, 0);
    public Scalar leftInclusion = new Scalar(0, 0, 0);
    public Scalar rightInclusion = new Scalar(0, 0, 0);

    public YellowPipeline(AprilTagProcessor aprilTag, int targetAprilTagID, boolean diditsee) {
        this.aprilTag = aprilTag;
        this.targetAprilTagID = targetAprilTagID;
        this.diditsee = diditsee;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        aprilTag.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == targetAprilTagID && detection.metadata != null) {
                    int leftX = Integer.MAX_VALUE;
                    int rightX = Integer.MIN_VALUE;
                    int topY = Integer.MIN_VALUE;
                    int bottomY = Integer.MAX_VALUE;

                    diditsee = true;

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

//                    Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
//                    Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);

                    Imgproc.rectangle(frame, leftInclusionZone, rectColor, 2);
                    Imgproc.rectangle(frame, rightInclusionZone, rectColor, 2);

                    Mat leftInclusionZoneYellow = frame.submat(leftInclusionZone);
                    Mat rightInclusionZoneYellow = frame.submat(rightInclusionZone);

                    leftInclusion = Core.mean(leftInclusionZoneYellow);
                    rightInclusion = Core.mean(rightInclusionZoneYellow);

                    double leftInclusionPixels = (leftInclusion.val[0] + leftInclusion.val[2]) / 2;
                    double rightInclusionPixels = (rightInclusion.val[0] + rightInclusion.val[2]) / 2;

                    if (leftInclusionPixels > yellowTresholdRightSide && (leftInclusion.val[0] + leftInclusion.val[3]) > leftInclusionPixels) {
                        preloadedZone = left;
                    } else if (rightInclusionPixels > yellowTresholdRightSide && (rightInclusion.val[0] + rightInclusion.val[3]) > rightInclusionPixels) {
                        preloadedZone = right;
                    } else if ((leftInclusionPixels > yellowTresholdCenter && (leftInclusion.val[0] + leftInclusion.val[3]) > leftInclusionPixels) && (rightInclusionPixels > yellowTresholdCenter && (rightInclusion.val[0] + rightInclusion.val[3]) > rightInclusionPixels)) {
                        preloadedZone = center;
                    } else {
                        preloadedZone = none;
                    }

                    if(side == Side.blue)
                    {
                        if(preloadedZone == left)
                        { clawpoz = 3;}
                        else
                        {
                            clawpoz = 1;
                        }
                    }  else{
                        if(preloadedZone == left)
                        {
                            clawpoz = 1;
                        } else
                        {
                            clawpoz = 3;
                        }

                    }

                }
            }
        }
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}

//    public String whichSide(Mat frame, Rect inclusion, Rect exclusion){
//
//        double yellowTreshold = 150;
//
//        Scalar inclusionZoneColor = new Scalar(0, 0, 0);
//        Scalar exclusionZoneColor = new Scalar(0, 0 , 0);
//
//        Mat inclusionZone = frame.submat(inclusion);
//        Mat exclusionZone = frame.submat(exclusion);
//
//        inclusionZoneColor = Core.mean(inclusionZone);
//        exclusionZoneColor = Core.mean(exclusionZone);
//
//        double inclusionPixels = (inclusionZoneColor.val[0] + inclusionZoneColor.val[2]) / 2;
//
//        if(inclusionPixels > yellowTreshold && (inclusionZoneColor.val[0] + inclusionZoneColor.val[3]) < inclusionPixels){
//            preloadedZone = "left";
//        } else {
//            preloadedZone = "right";
//        }
//
//        return preloadedZone;
//
//    }

