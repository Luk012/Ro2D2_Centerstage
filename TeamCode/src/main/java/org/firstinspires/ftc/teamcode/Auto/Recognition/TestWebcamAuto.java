//package org.firstinspires.ftc.teamcode.Auto.Recognition;
//
//
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name="test webcam", group = "auto")
//public class TestWebcamAuto extends LinearOpMode {
//
//    enum STATUS {
//        DETECT_TEAM_PROP,
//        DETECT_YELLOW_PIXEL,
//        DETECT_APRIL_TAG,
//        ALIGN_TO_APRIL_TAG,
//        NOTHING,
//    }
//
//    final double DESIRED_DISTANCE = 12.0;
//
//    private static final boolean USE_WEBCAM = true;
//    private static final int DESIRED_TAG_ID = 6;
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection desiredTag = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        OpenCVMaster redCase = new OpenCVMaster(this);
//        redCase.observeStick();
//
//        String caz = redCase.opencv.getWhichSide();
//
//        boolean targetFound = false;
//
//        STATUS status = STATUS.DETECT_TEAM_PROP;
//
//        telemetry.addData("case", caz);
//        telemetry.update();
//        sleep(50);
//
//        waitForStart();
//
//        while(opModeIsActive() && !isStopRequested()){
//
//            switch (status){
//
//                case DETECT_TEAM_PROP:
//                {
//                    if(caz == "left"){
//                        status = STATUS.DETECT_YELLOW_PIXEL;
//                    } else if (caz == "center") {
//                        status = STATUS.DETECT_YELLOW_PIXEL;
//                    } else {
//                        status = STATUS.DETECT_YELLOW_PIXEL;
//                    }
//                    break;
//                }
//
//                case DETECT_YELLOW_PIXEL:
//                {
//                    YellowPixelMaster yellowPixel = new YellowPixelMaster(this);
//                    yellowPixel.observeStick();
//
//                    String yellowCase = yellowPixel.yellowPixel.getWhichSide();
//
//                    telemetry.addData("yellowCase", yellowCase);
//                    telemetry.update();
//                    status = STATUS.DETECT_APRIL_TAG;
//
//                    break;
//                }
//
//                case DETECT_APRIL_TAG:
//                {
//                    initAprilTag();
//                    setManualExposure(6, 250);
//
//                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//                    for (AprilTagDetection detection : currentDetections) {
//
//                        if (detection.metadata != null) {
//
//                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                                targetFound = true;
//                                desiredTag = detection;
//                                break;
//                            }
//                        }
//                    }
//
//                    status = STATUS.ALIGN_TO_APRIL_TAG;
//                    break;
//                }
//
//                case ALIGN_TO_APRIL_TAG:
//                {
//                    double rangeError;
//                    double headingError;
//                    double yawError;
//
//                    if(targetFound) {
//                        rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//                        headingError    = desiredTag.ftcPose.bearing;
//                        yawError        = desiredTag.ftcPose.yaw;
//                    }
//
//                    TrajectorySequence ALIGN_TO_APRIL_TAG = drive.trajectorySequenceBuilder(/* pozitie de unde ai ramas */)
//                            .lineToLinearHeading(new Pose2d(/* x de unde am ramas / + rangeError, / y de unde am ramas / + headingError, Math.toRadians(/ unghiul de unde am ramas */ + yawError)))
//                            .build();
//
//                    drive.followTrajectorySequenceAsync(ALIGN_TO_APRIL_TAG);
//                    status = NOTHING;
//                    break;
//                }
//
//            }
//
//        }
//
//    }
//
//    private void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        // Create the vision portal by using a builder.
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .build();
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(aprilTag)
//                    .build();
//        }
//    }
//
//    private void    setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
////        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
////            telemetry.addData("Camera", "Waiting");
////            telemetry.update();
////            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
////                sleep(20);
////            }
////            telemetry.addData("Camera", "Ready");
////            telemetry.update();
////        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//    }
//
//}