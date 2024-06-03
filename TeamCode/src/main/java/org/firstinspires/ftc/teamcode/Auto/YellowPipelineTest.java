package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Recognition.Globals;
import org.firstinspires.ftc.teamcode.Auto.Recognition.Location;
import org.firstinspires.ftc.teamcode.Auto.Recognition.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "YellowPipelineTest")
public class YellowPipelineTest extends LinearOpMode {

    private YellowPipeline yellowPipeline;

    @Override
    public void runOpMode(){

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        boolean diditsee = false;
//
        yellowPipeline = new YellowPipeline(aprilTag, 5, diditsee);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .addProcessor(yellowPipeline)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Status", "Initialized");
            telemetry.addData("location", yellowPipeline.preloadedZone);
//            telemetry.addData("right zone average minim: ", (yellowPipeline.rightInclusion.val[0] + yellowPipeline.rightInclusion.val[2]) / 2);
//            telemetry.addData("left zone average minim: ", (yellowPipeline.leftInclusion.val[0] + yellowPipeline.leftInclusion.val[2]) / 2);
//            telemetry.addData("right zone average maxim: ", (yellowPipeline.rightInclusion.val[0] + yellowPipeline.rightInclusion.val[3]) / 2);
//            telemetry.addData("left zone average maxim: ", (yellowPipeline.leftInclusion.val[0] + yellowPipeline.leftInclusion.val[3]) / 2);
            telemetry.update();

        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Initialized");
            telemetry.addData("location", yellowPipeline.preloadedZone);
            telemetry.addData("right zone average minim: ", (yellowPipeline.rightInclusion.val[0] + yellowPipeline.rightInclusion.val[2]) / 2);
            telemetry.addData("left zone average minim: ", (yellowPipeline.leftInclusion.val[0] + yellowPipeline.leftInclusion.val[2]) / 2);
            telemetry.addData("right zone average maxim: ", (yellowPipeline.rightInclusion.val[0] + yellowPipeline.rightInclusion.val[3]) / 2);
            telemetry.addData("left zone average maxim: ", (yellowPipeline.leftInclusion.val[0] + yellowPipeline.leftInclusion.val[3]) / 2);
            telemetry.update();
            sleep(100);

        }

        visionPortal.close();
    }
}