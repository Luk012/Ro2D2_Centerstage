//package org.firstinspires.ftc.teamcode.Auto.P2P;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@Photon
//@TeleOp(name="testpidtopoint", group="testpidtopoint")
//public class testpidtopoint extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        funnypidtopoint controller = new funnypidtopoint(hardwareMap);
//
//         double x_start = -43, y_start = 61, angle_start = 90;
//         double x_purple_preload_center = -54, y_purple_preload_center = 21, angle_purple_preload_center = 174;
//        Pose purpleCenter = new Pose(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
//
//        Pose targetPose = new Pose(0, -50, Math.toRadians(0));
////        Pose currentPose = new Pose(0, 0, Math.toRadians(0));
//        Pose start_pose = new Pose(x_start, y_start,Math.toRadians(angle_start));
//
////        Pose targetPose2 = new Pose(0, 0, Math.toRadians(0));
//
//        controller.drive.setPose(start_pose);
//
//        waitForStart();
//
//
//
//        while (opModeIsActive()) {
//
//
//           // Pose2d currentpose = controller.drive.getPoseEstimate();
//            controller.execute(purpleCenter);
//
////            if(controller.isFinished(targetPose))
////            {
////                controller.execute(targetPose2);
////            }
//
//
////            double errorX = targetPose.getX() - currentpose.getX();
////            double errorY = targetPose.getY() - currentpose.getY();
////            double errorHeading = targetPose.heading - currentpose.getHeading();
//
//            // Update telemetry with the current pose and error values
//           // telemetry.addData("Current Pose", currentpose.toString());
//            telemetry.addData("X",  controller.drive.returnPose().x);
//            telemetry.addData("Y",  controller.drive.returnPose().y);
//            telemetry.addData("angle", Math.toDegrees(controller.drive.returnPose().heading) );
//            telemetry.addData("hasReachedProximity", controller.hasReachedProximity);
//           // telemetry.addData("Error Heading (degrees)", Math.toDegrees(errorHeading));
//            telemetry.update();
//
//          //  controller.drive.update();
//            // Optional: Adjust the loop frequency if necessary
//            sleep(50);
//        }
//
//        // Optionally, stop motors after the movement is finished
//      //  controller.stopMotors();
//
//        // Indicate that the target has been reached
//        telemetry.addData("Status", "Target Reached");
//        telemetry.update();
//    }
//}
