package org.firstinspires.ftc.teamcode.Auto.Recognition;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.Recognition.AtagYellow;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "AtagYellowOpMode", group = "Concept")
public class AtagYellowOpMode extends LinearOpMode {

    private YellowPipeline atagYellow ;
    public AtagMath atagMath;
    public static int desieredTag = 5;

    public static int duration =5;
    public static int gainControl = 255;
    public static Pose2d pose2d = new Pose2d(50, 50, Math.toRadians(180));


    @Override
    public void runOpMode() {



        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(520.035, 520.035, 288.093, 269.186)

        .build();

        atagYellow = new YellowPipeline(aprilTagProcessor, desieredTag);


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(atagYellow)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(duration, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);

        gain.setGain(gainControl);

        // Wait for start button to be pressed
        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Status", "Initialized");
          //  telemetry.addData("location", atagYellow.location);
            telemetry.addData("right zone average: ", atagYellow.rightZoneAverage);
            telemetry.addData("left zone average: ", atagYellow.leftZoneAverage);
            try {
                telemetry.addData("H = " , atagYellow.averageHSV[0]);
                telemetry.addData("S = ", atagYellow.averageHSV[1]);
                telemetry.addData("V = ", atagYellow.averageHSV[2]);
            } catch (Exception e) {
                e.printStackTrace();
            }
//            telemetry.addData("does it see", atagYellow.doesitseedesieredatag);
//            telemetry.addData("Location", atagYellow.location);
            telemetry.update();
           // visionPortal.stopStreaming();

        }

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detectionList = aprilTagProcessor.getDetections();

            for (AprilTagDetection detection : detectionList) {
                if (detection.id == 2) {
                    telemetry.addData("pose x", atagYellow.poseFromTag(pose2d, detection).getX());
                    telemetry.addData("pose y", atagYellow.poseFromTag(pose2d, detection).getY());
                    telemetry.addData("pose angle", Math.toDegrees(atagYellow.CorectedDetecion(detection).getHeading()));
                }}
//            telemetry.addData("Location", atagYellow.location);
         //  telemetry.addData("location", atagYellow.location);
//            telemetry.addData("right zone average: ", atagYellow.rightZoneAverage);
//            telemetry.addData("left zone average: ", atagYellow.leftZoneAverage);
            telemetry.update();
            sleep(100);


        }

        // Stop the camera
        visionPortal.close();
    }
}