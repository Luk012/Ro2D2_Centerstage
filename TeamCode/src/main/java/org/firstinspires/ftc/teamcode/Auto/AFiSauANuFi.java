//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.imgproc.Imgproc;
//
//import java.util.List;
//
//public class AFiSauANuFi extends LinearOpMode {
//
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        while(opModeIsActive()){
//            aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//
//
//
////        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            if (currentDetections != null) {
//                for (AprilTagDetection detection : currentDetections) {
//                    if (detection.metadata != null) {
//                        if (detection.id == targetAprilTagID) {
//                            int leftX = Integer.MAX_VALUE;
//                            int rightX = Integer.MIN_VALUE;
//                            int topY = Integer.MIN_VALUE;
//                            int bottomY = Integer.MAX_VALUE;
//                            isDetected = true;
//
//                            for (Point point : detection.corners) {
//                                if (point.x < leftX) leftX = (int) point.x;
//                                if (point.x > rightX) rightX = (int) point.x;
//                                if (point.y > topY) topY = (int) point.y;
//                                if (point.y < bottomY) bottomY = (int) point.y;
//                            }
//
//                            int tagCenterX = (int) detection.center.x;
//                            int tagCenterY = (int) detection.center.y;
//
//                            int tagWidth = rightX - leftX;
//                            int tagHeight = topY - bottomY;
//
//                            int inclusionZoneWidth = (int) (tagWidth * 1.5);
//                            int inclusionZoneHeight = (int) (tagHeight * 1.5);
//
//                            int exclusionZoneWidth = (int) (tagWidth * 0.28);
//                            int exclusionZoneHeight = (int) (tagHeight * 0.28);
//
//                            Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
//                            Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
//
//                            Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.64), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);
//                            Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.28), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);
//
//                            Imgproc.rectangle(frame, leftInclusionZone, rectColor, 1);
//                            Imgproc.rectangle(frame, rightInclusionZone, rectColor, 1);
//
//                            int leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
//                            int rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);
//
//                        }
//    }
//
//    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
//        if (frame == null) {
////            System.out.println("frame is bad");
//            return 0;
//        }
//
//        int sum = 0;
//        int count = 0;
//        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
//            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
//                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
//                    continue;
//                }
//
//                if (x >= exclusionRect.x && x < exclusionRect.x + exclusionRect.width && y >= exclusionRect.y && y < exclusionRect.y + exclusionRect.height) {
//                    continue;
//                }
//
//                double[] data = frame.get(y, x);
//                if (data != null && data.length > 0) {
//                    sum += data[0];
//                    count++;
//                }
//            }
//        }
//
//        return count > 0 ? sum / count : 0;
//    }
//}
