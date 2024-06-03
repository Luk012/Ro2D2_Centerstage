package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline.Side.blue;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "test outtake")
public class TestOuttakeYellow extends LinearOpMode {

    private YellowPipeline yellowPipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        robotMap r = new robotMap(hardwareMap);
        boolean diditsee = false;

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();



        yellowPipeline = new YellowPipeline(aprilTag, 5, diditsee);
        yellowPipeline.side = blue;

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .addProcessor(yellowPipeline)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        clawAngleController clawAngle = new clawAngleController();
        clawFlipController clawFlip = new clawFlipController();
        fourbarController fourbar = new fourbarController();
        clawAngle.clawAngle_i = 1;
        fourbar.CS = fourbarController.fourbarStatus.SCORE;
        clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
        clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;

        clawAngle.update(r);
        clawFlip.update(r);
        fourbar.update(r);

        while (!isStopRequested() && !isStarted()){

            telemetry.addData("Status", "Initialized");
            telemetry.addData("location", yellowPipeline.preloadedZone);
            telemetry.update();

        }

        waitForStart();

        while (opModeIsActive()){

            if(diditsee == true)
            {
                visionPortal.stopStreaming();
            }

                switch (yellowPipeline.preloadedZone) {
                    case left:
                        clawAngle.clawAngle_i = yellowPipeline.clawpoz;
                        break;
                    case right:
                        clawAngle.clawAngle_i = yellowPipeline.clawpoz;
                        break;
                    case center:
                        clawAngle.clawAngle_i = yellowPipeline.clawpoz;
                        break;
                    default:
                        clawAngle.clawAngle_i = yellowPipeline.clawpoz;
                        break;
                }



            clawAngle.update(r);
            clawFlip.update(r);
            fourbar.update(r);

            telemetry.addData("Status", "Initialized");
            telemetry.addData("location", yellowPipeline.preloadedZone);
            telemetry.update();

        }

    }
}
