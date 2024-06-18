package org.firstinspires.ftc.teamcode.Auto.Recognition;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class AtagMath {
    public static double distanceToCenter = 18.6 / 2.54;
    public static double AprilTagToINCHES = 25.4; // assume distance is in meters
    public static double[] TAG_X_OFFSET = {0, 59.8, 59.8, 59.8, 59.8, 59.8, 59.8};
    public static double[] TAG_Y_OFFSET = {0, 39.8, 39.8, 39.8, 39.8, 33.8, 27.8};
    public static Pose2d poseFromTag(Pose2d robotPose, AprilTagDetection detection) {
        double tagX = detection.rawPose.z;
        double tagY = detection.rawPose.x;

        double robotHeading = robotPose.getHeading();

        double x_displacement = cos(robotHeading) * tagX - sin(robotHeading) * tagY;
        double y_displacement = cos(robotHeading) * tagY + sin(robotHeading) * tagX;

        double x_to_center = cos(robotHeading) * distanceToCenter;
        double y_to_center = sin(robotHeading) * distanceToCenter;

        return new Pose2d((x_displacement + x_to_center) + TAG_X_OFFSET[detection.id], -(y_displacement + y_to_center) + TAG_Y_OFFSET[detection.id], robotHeading);
    }
}