package org.firstinspires.ftc.teamcode.Auto.Recognition;

import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.location.nothing;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Config
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

    public int targetAprilTagID = -1;
    public int yellowTresholdLeftSide = 120; // treshold when the robot is in the left side / in front of the AprilTag
    public int yellowTresholdRightSide = 85; // treshold when the robot is in the right side of the AprilTag; decreases to the right
    public int yellowTresholdCenter = 80; // treshold when the pixel is on top of the AprilTag (center case)

    public static double distanceToCenter = 19.5 / 2.54;
    public static double AprilTagToINCHES = 25.4; // assume distance is in meters
    public static double[] TAG_X_OFFSET = {0, 35.5, 35.5, 35.5, 57.7, 59, 57.7};
    public static double[] TAG_Y_OFFSET = {0, 41, 35, 27.5, 40.9, 39.2, 40.9};

    public location preloadedZone = nothing; //default
    public boolean diditsee = false;
   public Side side = Side.nothing;
    public int clawpoz = 0;
    public int  leftZoneAverage, rightZoneAverage;
    private double absolute_diff;

    public static double lowhue = 70, lowsaturation = 138, lowvalue = 140;
    public static double highhue = 110, highsaturation = 178, highvalue = 255;

    public double[] averageHSV;

    private AprilTagProcessor aprilTag;

    public Scalar rectColor = new Scalar(0, 255, 0);
    public Scalar leftInclusion = new Scalar(0, 0, 0);
    public Scalar rightInclusion = new Scalar(0, 0, 0);

    public YellowPipeline(AprilTagProcessor aprilTag, int targetAprilTagID) {
        this.aprilTag = aprilTag;
        this.targetAprilTagID = targetAprilTagID;

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        aprilTag.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        try{
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == Globals.desieredtag && detection.metadata != null) {
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

                    leftZoneAverage = meanIntensity(frame, leftInclusionZone);
                    rightZoneAverage = meanIntensity(frame, rightInclusionZone);

                    Globals.aprilTagDetection = detection;

                    absolute_diff = Math.abs(leftZoneAverage - rightZoneAverage);

                     averageHSV = calculateAverageHSV(frame, leftInclusionZone);

                    if (leftZoneAverage <= 150 && rightZoneAverage <= 150) {
                        Globals.side = Globals.Side.NONE;
                        Globals.isItNone = true;
                    } else if(leftZoneAverage > 250 && rightZoneAverage > 250 ){
                        Globals.side = Globals.Side.MIDDLE;
                        Globals.isItMiddle = true;
                    } else if(leftZoneAverage > 500 && rightZoneAverage <= 250)
                    {
                        Globals.side = Globals.Side.LEFT;
                    } else
                    {
                        Globals.side = Globals.Side.RIGHT;
                    }

                    Globals.returnAutoClawPoz(Globals.alliance, Globals.side);

                    diditsee = true;
                }
            }
        }}catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public int meanIntensity(Mat frame, Rect inclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        Mat goodarea = new Mat(frame, inclusionRect);

        Mat hsvImage = new Mat();
        Imgproc.cvtColor(goodarea, hsvImage, Imgproc.COLOR_BGR2HSV);

        Scalar lowerYellow = new Scalar(lowhue, lowsaturation, lowvalue);
        Scalar upperYellow = new Scalar(highhue, highsaturation, highvalue);

        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerYellow, upperYellow, mask);

        int yellowCount = Core.countNonZero(mask);



        return yellowCount;

    }

    public static double[] calculateAverageHSV(Mat image, Rect rect) {
        // Crop the image to the specified rectangle
        Mat croppedImage = new Mat(image, rect);

        // Convert the image to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(croppedImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Calculate the sum of HSV values
        double sumH = 0, sumS = 0, sumV = 0;
        int pixelCount = 0;

        for (int row = 0; row < hsvImage.rows(); row++) {
            for (int col = 0; col < hsvImage.cols(); col++) {
                double[] hsv = hsvImage.get(row, col);
                sumH += hsv[0];
                sumS += hsv[1];
                sumV += hsv[2];
                pixelCount++;
            }
        }

        // Calculate the average HSV values
        double averageH = sumH / pixelCount;
        double averageS = sumS / pixelCount;
        double averageV = sumV / pixelCount;

        return new double[]{averageH, averageS, averageV};
    }

    public static Pose2d poseFromTag(Pose2d robotPose, AprilTagDetection detection) {
        double tagX = detection.rawPose.z;
        double tagY = detection.rawPose.x;

        double robotHeading = robotPose.getHeading();

        double x_displacement = cos(robotHeading) * tagX - sin(robotHeading) * tagY;
        double y_displacement = cos(robotHeading) * tagY + sin(robotHeading) * tagX;

        double x_to_center = cos(robotHeading) * distanceToCenter;
        double y_to_center = sin(robotHeading) * distanceToCenter;

       // return new Pose2d((x_displacement + x_to_center) + TAG_X_OFFSET[detection.id], -(y_displacement + y_to_center) + TAG_Y_OFFSET[detection.id], robotHeading);
        return new Pose2d(robotPose.getX(), -(y_displacement + y_to_center) + TAG_Y_OFFSET[detection.id], robotHeading);


    }

    public static double getDetectionDistance(AprilTagDetection aprilTagDetection)
    {
        return Math.sqrt(
                aprilTagDetection.ftcPose.x * aprilTagDetection.ftcPose.x +
                aprilTagDetection.ftcPose.y * aprilTagDetection.ftcPose.y +
                aprilTagDetection.ftcPose.z * aprilTagDetection.ftcPose.z

        );
    }

    public static AprilTagPoseFtc getFtcPoseFromRawPose(AprilTagPoseRaw rawPose)
    {
        // taken from AprilTagProcessorImpl
        if (rawPose == null) return null;
        Orientation rot = Orientation.getOrientation(rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);

        return new AprilTagPoseFtc(
                rawPose.x,  // x   NB: These are *intentionally* not matched directly;
                rawPose.z,  // y       this is the mapping between the AprilTag coordinate
                -rawPose.y, // z       system and the FTC coordinate system
                -rot.firstAngle, // yaw
                rot.secondAngle, // pitch
                rot.thirdAngle,  // roll
                Math.hypot(rawPose.x, rawPose.z),  // range
                Math.atan2(-rawPose.x, rawPose.z), // bearing
                Math.atan2(-rawPose.y, rawPose.z)  // elevation
        );
    }

    public static Pose2d CorectedDetecion(AprilTagDetection detection)
    {
        double uncorrectedDistance = getDetectionDistance(detection);
        double correctedDistance = 0.843424 * uncorrectedDistance + 0.582304;
        double scale = correctedDistance / uncorrectedDistance;

            return new Pose2d(detection.ftcPose.x + TAG_X_OFFSET[detection.id], detection.ftcPose.y, Math.atan(detection.rawPose.x/detection.rawPose.y));

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

