package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.globals.robotMap;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@Photon
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {


    public static double x_start = 0, y_start = 0, angle_start = 0;
    double loopTime;
    Pose2d start_pose = new Pose2d(x_start, y_start, Math.PI);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotMap robotMap = new robotMap(hardwareMap);
        drive.setPoseEstimate(start_pose);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotMap.collect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotMap.extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        robotMap.collect.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotMap.extendoRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));


//            Pose2d relativePose = poseEstimate.minus(start_pose);
//            telemetry.addData("x relative", drive.getPoseEstimate().getX());
//            telemetry.addData("y relative", drive.getPoseEstimate().getY());
//            telemetry.addData("heading relative", Math.toDegrees(drive.getPoseEstimate().getHeading()));
          telemetry.addData("paralel", robotMap.collect.getCurrentPosition());
         telemetry.addData("perpendicular", robotMap.extendoRight.getCurrentPosition());
            loopTime = loop;
            telemetry.update();
        }
    }
}
