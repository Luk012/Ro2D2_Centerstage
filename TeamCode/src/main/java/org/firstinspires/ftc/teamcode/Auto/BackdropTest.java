package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Recognition.Globals;
import org.firstinspires.ftc.teamcode.Auto.Recognition.Location;
import org.firstinspires.ftc.teamcode.Auto.Recognition.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Disabled
@Autonomous(name = "BackdropTest")
public class BackdropTest extends LinearOpMode {

    private PreloadDetectionPipeline preloadPipeline;
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;

        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

//        preloadPipeline = new PreloadDetectionPipeline(aprilTag);

        preloadPipeline = new PreloadDetectionPipeline();
        preloadPipeline.setTargetAprilTagID(Location.CENTER);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessors(preloadPipeline, aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        while (opModeInInit()) {
            telemetry.addData("Side: ", preloadPipeline.getPreloadedZone());
            telemetry.addData("Target: ", preloadPipeline.getTargetAprilTagID());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Side: ", preloadPipeline.getPreloadedZone());
            telemetry.addData("Target: ", preloadPipeline.getTargetAprilTagID());
            telemetry.addData("isDetected", preloadPipeline.isDetected);
            telemetry.update();
        }
    }
}