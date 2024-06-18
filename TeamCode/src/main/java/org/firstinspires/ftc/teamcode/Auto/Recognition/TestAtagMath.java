//package org.firstinspires.ftc.teamcode.Auto.Recognition;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.openftc.apriltag.AprilTagDetection;
//
//@Autonomous(name = "Test AprilTag Math", group = "Test")
//public class TestAtagMath extends LinearOpMode {
//
//    p
//
//    @Override
//    public void runOpMode() {
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // Define a sample robot pose
//        Pose2d robotPose = new Pose2d(0, 0, 0); // Starting at the origin facing forward
//
//        // Create a sample AprilTagDetection
//        AprilTagDetection detection = new AprilTagDetection();
//
//
//
//        // Compute the robot pose from the AprilTag detection
//        Pose2d computedPose = AtagMath.poseFromTag(robotPose, detection);
//
//        // Display the results on the telemetry
//        telemetry.addData("Computed X", computedPose.getX());
//        telemetry.addData("Computed Y", computedPose.getY());
//        telemetry.addData("Computed Heading", computedPose.getHeading());
//        telemetry.update();
//
//        // Keep the telemetry visible for 10 seconds
//        sleep(10000);
//    }
//}
