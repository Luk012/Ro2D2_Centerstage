//package org.firstinspires.ftc.teamcode.Auto;
//
//
//
//import com.acmerobotics.dashboard.config.Config;
//
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
//import org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe;
//
//import org.firstinspires.ftc.teamcode.Auto.P2P.funnypidtopoint;
//import org.firstinspires.ftc.teamcode.Auto.Recognition.BluePipelineStackMaster;
//
//import org.firstinspires.ftc.teamcode.DistanceSensorCalibrator;
//
//import org.firstinspires.ftc.teamcode.globals.robotMap;
//
//import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
//import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
//import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
//import org.firstinspires.ftc.teamcode.system_controllers.doorController;
//import org.firstinspires.ftc.teamcode.system_controllers.droneController;
//import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
//import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
//import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
//import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
//import org.firstinspires.ftc.teamcode.system_controllers.liftController;
//
//import java.util.Arrays;
//import java.util.List;
//
//@Photon
//@Config
//@Autonomous(group = "Auto" , name = "BlueFarP2P")
//
//public class BlueFarP2P extends LinearOpMode {
//
//    enum STROBOT {
//        START,
//        PURPLE,
//        SYSTEMS_PURPLE,
//        COLLECT_PURPLE,
//        VERIF_TIMER,
//        GO_SCORE_YELLOW,
//        PREPARE_SCORE_YELLOW,
//        PREPARE_SCORE_YELLOW_v2,
//        VERIF_PURPLE_SCORE,
//        YELLOW_DROP,
//
//        PREPARE_COLLECT,
//        COLLECT_EXTENDO,
//        COLLECT_VERIF_PIXLES,
//        COLLECT_VERIF_PIXLES_V2,
//
//        GO_SCORE_CYCLE,
//        PREPARE_SCORE_CYCLE,
//        SCORE_CYCLE,
//        FAIL_SAFE,
//       FAIL_SAFE_HEADER_VERIF,
//        FAIL_SAFE_2,
//        FAIL_SAFE_HEADER_VERIF_2,
//        FAILSAFE_PURPLE,
//        RETRACT_AND_RERTY,
//        RETRY_TIMER_RESET,
//        RETRACT_AND_RERTY_v2,
//        RETRY_TIMER_RESET_v2,
//
//        PARK,
//
//
//        NOTHING
//    }
//
//    public static double x_start = -43, y_start = 61, angle_start = 90;
//
//    /**
//     * purple
//     */
//
//    public static double x_purple_preload_right = -43, y_purple_preload_right = 26, angle_purple_preload_right = 180;
//    public static double x_purple_preload_center = -49, y_purple_preload_center = 19, angle_purple_preload_center = 174;
//    public static double x_purple_preload_left = -63, y_purple_preload_left = 18, angle_purple_preload_left = 190;
//
//    /**
//     * yellow
//     */
//
//    public static double x_yellow_preload_right = 43, y_yellow_preload_right = 36.9, angle_yellow_preload_right = 180;
//    public static double x_yellow_preload_center = 43, y_yellow_preload_center = 30, angle_yellow_preload_center = 180;
//    public static double x_yellow_preload_left = 43, y_yellow_preload_left = 27, angle_yellow_preload_left = 180;
//   // public static double x_yellow_inter =
//
//    /**
//     * collect
//     */
//
//
//
//    // Cycle 2
//    public static double x_inter_collect_cycle_2_right = 30, y_inter_collect_cycle_2_right = 7, angle_inter_collect_cycle_2_right = 180;
//    public static double x_collect_cycle_2_right = -26, y_collect_cycle_2_right = 7, angle_collect_cycle_2_right = 180;
//
//    public static double x_inter_collect_cycle_2_center = 27, y_inter_collect_cycle_2_center = 7, angle_inter_collect_cycle_2_center = 180;
//    public static double x_collect_cycle_2_center = -21, y_collect_cycle_2_center = 7, angle_collect_cycle_2_center = 180;
//
//    public static double x_inter_collect_cycle_2_left = 30, y_inter_collect_cycle_2_left = 6.5, angle_inter_collect_cycle_2_left = 180;
//    public static double x_collect_cycle_2_left = -27.2, y_collect_cycle_2_left = 6.5, angle_collect_cycle_2_left = 180;
//
//    // Cycle 3
//    public static double x_inter_collect_cycle_3_right = 30, y_inter_collect_cycle_3_right = 6, angle_inter_collect_cycle_3_right = 180;
//    public static double x_collect_cycle_3_right = -26, y_collect_cycle_3_right = 7, angle_collect_cycle_3_right = 180;
//
//    public static double x_inter_collect_cycle_3_center = 27, y_inter_collect_cycle_3_center = 7, angle_inter_collect_cycle_3_center = 180;
//    public static double x_collect_cycle_3_center = -21, y_collect_cycle_3_center = 7, angle_collect_cycle_3_center = 180;
//
//    public static double x_inter_collect_cycle_3_left = 30, y_inter_collect_cycle_3_left = 7, angle_inter_collect_cycle_3_left = 180;
//    public static double x_collect_cycle_3_left = -26, y_collect_cycle_3_left = 7, angle_collect_cycle_3_left = 180;
//
//    public static double retry_x = -30.5, retry_y = 6, retry_angle = 180;
//
//    /**
//     * score
//     */
//
//
//    // Second cycle scoring positions
//    public static double x_score_second_cycle_right = 42, y_score_second_cycle_right = 9.5, angle_score_second_angle_right = 210;
//    public static double x_score_second_cycle_center = 46, y_score_second_cycle_center = 10, angle_score_second_angle_center = 210;
//    public static double x_score_second_cycle_left = 42, y_score_second_cycle_left = 11, angle_score_second_angle_left = 210;
//
//    // Third cycle scoring positions
//    public static double x_score_third_cycle_right = 42, y_score_third_cycle_right = 10, angle_score_third_angle_right = 210;
//    public static double x_score_third_cycle_center = 42, y_score_third_cycle_center = 10, angle_score_third_angle_center = 210;
//    public static double x_score_third_cycle_left = 42, y_score_third_cycle_left = 10, angle_score_third_angle_left = 210;
//
//
//
//
//
//
//    /**
//     * intern collect
//     */
//
//    public static double x_inter_collect_first_cycle = -44, y_inter_collect_first_cycle = 7, angle_inter_collect_first_cycle = 180;
//
//    public static double x_inter_collect_first_cycle_left = -61, y_inter_collect_first_cycle_left = 7, angle_inter_collect_first_cycle_left = 180;
//
//    public static double x_inter_collect_first_cycle_center = -57.5, y_inter_collect_first_cycle_center = 7, angle_inter_collect_first_cycle_center = 180;
//
//    /**
//     * intern score
//     */
//
//    public static double x_inter_score_first_cycle = 23, y_inter_score_first_cycle = 7, angle_inter_score_first_cycle =180;
//    public static double x_inter_score_first_cycle_left = 23, y_inter_score_first_cycle_left = 7, angle_inter_score_first_cycle_left =180;
//
//    public static int caz = 0;
//    public static double limit = 1.6;
//    boolean forced = false;
//    public static int tries = 0;
//    public static int tries_purple = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        BluePipelineStackMaster blueRight = new BluePipelineStackMaster(this);
//        blueRight.observeStick();
//
//        DistanceSensorCalibrator calibrator;
//
//       // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        robotMap r = new robotMap(hardwareMap);
//
//        funnypidtopoint pidtopoint = new funnypidtopoint(hardwareMap);
//
//
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//
//        clawAngleController clawAngle = new clawAngleController();
//        clawFlipController clawFlip = new clawFlipController();
//        collectAngleController collectAngle = new collectAngleController();
//        doorController door = new doorController();
//        fourbarController fourbar = new fourbarController();
//        latchLeftController latchLeft = new latchLeftController();
//        latchRightController latchRight = new latchRightController();
//        droneController drone = new droneController();
//        liftController lift = new liftController();
//        extendoController extendo = new extendoController();
//        failsafe failsafecontroller = new failsafe();
//
//        RedFarAutoController redFarAutoController = new RedFarAutoController();
//
//
//        double voltage;
//        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//        voltage = batteryVoltageSensor.getVoltage();
//
//        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
//        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
//        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
//        door.CS = doorController.doorStatus.CLOSED;
//        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
//        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
//        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
//        lift.CS = liftController.liftStatus.INITIALIZE;
//        extendo.CS = extendoController.extendoStatus.RETRACTED;
//        failsafecontroller.CurrentStatus = failsafe.failsafeStatus.NOTHING;
//        // angle mai mare la al treilea ciclu
////
//      //  funnypidtopoint.drive.update();
//        clawAngle.update(r);
//        clawFlip.update(r);
//        door.update(r);
//        fourbar.update(r);
//        latchLeft.update(r);
//        latchRight.update(r);
//        drone.update(r);
//        lift.update(r, 0, voltage);
//        extendo.update(r, 0, 1, voltage);
//        redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
//        failsafecontroller.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
//
//        collectAngle.update(r);
//
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//
//        }
//
//        Pose start_pose = new Pose(x_start, y_start,Math.toRadians(angle_start));
//
//        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
//        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
//        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));
//
//        Pose interCollectFirstCycle = new Pose(x_inter_collect_first_cycle, y_inter_collect_first_cycle, Math.toRadians(angle_inter_collect_first_cycle));
//        Pose interScoreFirstCycle = new Pose(x_inter_score_first_cycle, y_inter_score_first_cycle, Math.toRadians(angle_inter_score_first_cycle));
//
//        Pose interCollectFirstCycleleft = new Pose(x_inter_collect_first_cycle_left, y_inter_collect_first_cycle_left, Math.toRadians(angle_inter_collect_first_cycle_left));
//        Pose interScoreFirstCycleleft = new Pose(x_inter_score_first_cycle_left, y_inter_score_first_cycle_left, Math.toRadians(angle_inter_score_first_cycle_left));
//
//        Pose interCollectFirstCycleCenter = new Pose(x_inter_collect_first_cycle_center, y_inter_collect_first_cycle_center, Math.toRadians(angle_inter_collect_first_cycle_center));
//
//        Pose yellowRight = new Pose(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
//        Pose yellowCenter = new Pose(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
//        Pose yellowLeft = new Pose(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));
//
//        // Cycle 2 Right
//        Pose collect_inter_cycle2_right = new Pose(x_inter_collect_cycle_2_right, y_inter_collect_cycle_2_right, Math.toRadians(angle_inter_collect_cycle_2_right));
//        Pose collect_cycle2_right = new Pose(x_collect_cycle_2_right, y_collect_cycle_2_right, Math.toRadians(angle_collect_cycle_2_right));
//
//// Cycle 2 Center
//        Pose collect_inter_cycle2_center = new Pose(x_inter_collect_cycle_2_center, y_inter_collect_cycle_2_center, Math.toRadians(angle_inter_collect_cycle_2_center));
//        Pose collect_cycle2_center = new Pose(x_collect_cycle_2_center, y_collect_cycle_2_center, Math.toRadians(angle_collect_cycle_2_center));
//
//// Cycle 2 Left
//        Pose collect_inter_cycle2_left = new Pose(x_inter_collect_cycle_2_left, y_inter_collect_cycle_2_left, Math.toRadians(angle_inter_collect_cycle_2_left));
//        Pose collect_cycle2_left = new Pose(x_collect_cycle_2_left, y_collect_cycle_2_left, Math.toRadians(angle_collect_cycle_2_left));
//
//// Cycle 3 Right
//        Pose collect_inter_cycle3_right = new Pose(x_inter_collect_cycle_3_right, y_inter_collect_cycle_3_right, Math.toRadians(angle_inter_collect_cycle_3_right));
//        Pose collect_cycle3_right = new Pose(x_collect_cycle_3_right, y_collect_cycle_3_right, Math.toRadians(angle_collect_cycle_3_right));
//
//// Cycle 3 Center
//        Pose collect_inter_cycle3_center = new Pose(x_inter_collect_cycle_3_center, y_inter_collect_cycle_3_center, Math.toRadians(angle_inter_collect_cycle_3_center));
//        Pose collect_cycle3_center = new Pose(x_collect_cycle_3_center, y_collect_cycle_3_center, Math.toRadians(angle_collect_cycle_3_center));
//
//// Cycle 3 Left
//        Pose collect_inter_cycle3_left = new Pose(x_inter_collect_cycle_3_left, y_inter_collect_cycle_3_left, Math.toRadians(angle_inter_collect_cycle_3_left));
//        Pose collect_cycle3_left = new Pose(x_collect_cycle_3_left, y_collect_cycle_3_left, Math.toRadians(angle_collect_cycle_3_left));
//
//
//
//        // Second cycle scoring positions
//        Pose score_second_cycle_right = new Pose(x_score_second_cycle_right, y_score_second_cycle_right, Math.toRadians(angle_score_second_angle_right));
//        Pose score_second_cycle_center = new Pose(x_score_second_cycle_center, y_score_second_cycle_center, Math.toRadians(angle_score_second_angle_center));
//        Pose score_second_cycle_left = new Pose(x_score_second_cycle_left, y_score_second_cycle_left, Math.toRadians(angle_score_second_angle_left));
//
//// Third cycle scoring positions
//        Pose score_third_cycle_right = new Pose(x_score_third_cycle_right, y_score_third_cycle_right, Math.toRadians(angle_score_third_angle_right));
//        Pose score_third_cycle_center = new Pose(x_score_third_cycle_center, y_score_third_cycle_center, Math.toRadians(angle_score_third_angle_center));
//        Pose score_third_cycle_left = new Pose(x_score_third_cycle_left, y_score_third_cycle_left, Math.toRadians(angle_score_third_angle_left));
//
//        Pose retrypose = new Pose(retry_x, retry_y, Math.toRadians(retry_angle));
//
//        Pose parkpose = new Pose(40, 4, Math.toRadians(180));
//
//        List<Pose> yellow_right = Arrays.asList(
//               interCollectFirstCycle,
//                interScoreFirstCycle,
//               yellowRight
//
//        );
//
//        List<Pose> yellow_left = Arrays.asList(
//                interCollectFirstCycleleft,
//                interScoreFirstCycleleft,
//                yellowLeft
//
//        );
//
//        List<Pose> yellow_center = Arrays.asList(
//                interCollectFirstCycleCenter,
//                interScoreFirstCycle,
//                yellowCenter
//
//        );
//
//        List<Pose> collect_second_cycle = Arrays.asList(
//                collect_inter_cycle2_center,
//                collect_cycle2_center
//        );
//
//        List<Pose> collect_third_cycle = Arrays.asList(
//                collect_cycle3_center
//        );
//
//
//
//
//
//        STROBOT status = STROBOT.START;
//
//
//        int nrcicluri = 0;
//        double loopTime = 0;
//        double extendo_timer_i[] = {1.9, 1.1, 3};
//
//        double extendo_timer_i_purple[] = {1.2, 2.1, 1.2};
//
//        ElapsedTime transfer = new ElapsedTime();
//        ElapsedTime prepare_score_yellowqe = new ElapsedTime();
//        ElapsedTime prepare_collect = new ElapsedTime();
//        ElapsedTime extendo_timer = new ElapsedTime();
//        ElapsedTime verif_pixels = new ElapsedTime();
//        ElapsedTime preload = new ElapsedTime();
//        ElapsedTime verif = new ElapsedTime();
//        ElapsedTime score = new ElapsedTime();
//        ElapsedTime failsafe = new ElapsedTime();
//        ElapsedTime failsafe2 = new ElapsedTime();
//        ElapsedTime header = new ElapsedTime();
//        ElapsedTime park = new ElapsedTime();
//        ElapsedTime park_systems = new ElapsedTime();
//        ElapsedTime failsafe_purple = new ElapsedTime();
//        ElapsedTime retry = new ElapsedTime();
//
//        extendo.caz = 0;
//        collectAngle.collectAngle_i = 4;
//        lift.i_up = 0;
//        tries = 0;
//        tries_purple =0;
//
//        while (!isStarted() && !isStopRequested()) {
//
//            sleep(20);
//            if(blueRight.opencvstack.getWhichSide() == "left"){
//                caz = 0;
//            } else if (blueRight.opencvstack.getWhichSide() == "center") {
//                caz = 1;
//            } else {
//                caz = 2;
//            }
//            telemetry.addData("case", blueRight.opencvstack.getWhichSide());
//            telemetry.update();
//            sleep(50);
//        }
//
//       // funnypidtopoint.drive.setPose(start_pose);
//
//        waitForStart();
//park.reset();
////        double[] rawReadings = {28.5, 27.3, 26.2, 24.9, 24.3, 23.6, 28.4, 29.5, 30.6, 31.2, 31.3, 31.6, 32.3, 33.4, 34.5, 35.3, 36.1, 37.8, 37.9, 38};
////        double[] actualDistances = {23, 22, 21.5, 21, 20.5, 20, 22.5, 24, 24.4, 24.6, 25.2, 25.5, 26, 26.6, 27.2, 28.1, 28.9, 29.6, 30, 30.5};
////        calibrator = new DistanceSensorCalibrator(rawReadings, actualDistances);
//
//
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            int position = r.lift.getCurrentPosition();
//            int extendopos = r.extendoLeft.getCurrentPosition();
//
//
//            MotorConfigurationType motorConfigurationType = r.extendoRight.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.extendoRight.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.extendoLeft.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.extendoLeft.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.lift.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.lift.setMotorType(motorConfigurationType);
//
//            //  extendo.distance= r.extendoDistance.getDistance(DistanceUnit.MM);
//
//            switch (status) {
//
//                case START: {
//
//                    switch (caz)
//                    {
//                        case 0:
//                        {
//                           funnypidtopoint.setTargetPose(purpleRight);
//                            extendo.caz = 0;
//                            break;
//                        }
//                        case 1:
//                        {
//                         pidtopoint.setTargetPose(purpleCenter);
//                            extendo.caz = 1;
//                            break;
//
//                        }
//                        case 2:
//                        {
//                            funnypidtopoint.setTargetPose(purpleLeft);
//                            extendo.caz = 2;
//                            break;
//                        }
//                    }
//
//                    preload.reset();
//                    status = STROBOT.SYSTEMS_PURPLE;
//                    break;
//                }
//
//                case SYSTEMS_PURPLE:
//                {
//                    if(preload.seconds() > 0.1)
//                    {
//                        blueRight.stopCamera();
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE;
//                        extendo_timer.reset();
//                        status = STROBOT.PURPLE;
//                    }
//                    break;
//                }
//
//                case PURPLE:
//                {
//                    if(extendo_timer.seconds() > extendo_timer_i_purple[caz])
//                    {
//                        r.collect.setPower(1);
//                        preload.reset();
//                        extendo.CS = extendoController.extendoStatus.PURPLE;
//                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROP;
//                        status = STROBOT.COLLECT_PURPLE;
//                    }
//                    break;
//                }
//
//                case COLLECT_PURPLE:
//                {
//                    if((!r.pixelLeft.getState() && !r.pixelRight.getState()))
//                    {
//
//                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                        extendo.CS = extendoController.extendoStatus.RETRACTED;
//                        verif.reset();
//                        status = STROBOT.GO_SCORE_YELLOW;
//                    } else if(preload.seconds() > 1 && tries <3 && (r.pixelLeft.getState() || r.pixelRight.getState()))
//                    {                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_PURPLE;
//
//                        status = STROBOT.FAILSAFE_PURPLE;
//                    } else if(tries_purple >= 3)
//                    {
//                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                        extendo.CS = extendoController.extendoStatus.RETRACTED;
//                        verif.reset();
//                        status = STROBOT.GO_SCORE_YELLOW;
//                    }
//                    break;
//
//                }
//
//                case FAILSAFE_PURPLE: {
//                    if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE_PURPLE)
//                    {
//                        tries_purple += 1;
//                        preload.reset();
//                        status = STROBOT.COLLECT_PURPLE;
//                    }
//                    break;
//                }
//
//                case GO_SCORE_YELLOW:
//                {
//
//                    switch (caz)
//                    {
//                        case 0:
//
//                        {
//                                funnypidtopoint.setTargetPoses(yellow_right);
//                            if(verif.seconds() > 0.1)
//                            {
//                                r.collect.setPower(-1);
//                            }
//                            transfer.reset();
//                            status = STROBOT.PREPARE_SCORE_YELLOW;
//                            break;
//                        }
//                        case 1:
//                        {
//                                funnypidtopoint.setTargetPoses(yellow_center);
//                            if(verif.seconds() > 0.1)
//                            {
//                                r.collect.setPower(-1);
//                            }
//                            transfer.reset();
//                            status = STROBOT.PREPARE_SCORE_YELLOW;
//                            break;
//
//                        }
//                        case 2:
//                        {
//                                funnypidtopoint.setTargetPoses(yellow_left);
//                            if(verif.seconds() > 0.1)
//                            {
//                                r.collect.setPower(-1);
//                            }
//                            transfer.reset();
//                            status = STROBOT.PREPARE_SCORE_YELLOW;
//                            break;
//                        }
//                    }
//
//                    break;
//
//                }
//
//                case PREPARE_SCORE_YELLOW:
//                {
//
//                    if(transfer.seconds() > 0.5)
//                    {
//                        r.collect.setPower(0);
//                    }
//
//                    if(transfer.seconds() > 1.2)
//                    {
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
//                        prepare_score_yellowqe.reset();
//                        status = STROBOT.PREPARE_SCORE_YELLOW_v2;
//                    }
//                    break;
//                }
//
//                case PREPARE_SCORE_YELLOW_v2:
//                {
//                    if(funnypidtopoint.drive.getPoseEstimate().getX() >= 10 && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
//                    {
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN_BLUE;
//                        status = STROBOT.VERIF_PURPLE_SCORE;
//                    }
//
//                    break;
//                }
//
//                case VERIF_PURPLE_SCORE:
//                {
//                    if(funnypidtopoint.isSequenceFinished()  && redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE_BLUE)
//                    {
//
//                        status = STROBOT.YELLOW_DROP;
//                    }
//                    break;
//                }
//
//                case YELLOW_DROP:
//                {
////                    double rawReading = r.back.getDistance(DistanceUnit.CM);
////                    double calibratedDistance = calibrator.calibrate(rawReading);
//
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;
//
//
//                        prepare_collect.reset();
//                        status= STROBOT.PREPARE_COLLECT;
//
//                    break;
//                }
//
//                case PREPARE_COLLECT:
//                {
//                    if(prepare_collect.seconds() > 0.15)
//                    { switch (caz)
//                    {
//                        case 0:
//                        {
//                            funnypidtopoint.setTargetPoses(collect_second_cycle);
//                            break;
//                        }
//
//                        case 1:
//                        {
//                            funnypidtopoint.setTargetPoses(collect_second_cycle);
//                            break;
//                        }
//
//                        case 2:
//                        {
//                            funnypidtopoint.setTargetPoses(collect_second_cycle);
//                            break;
//                        }
//                    }}
//                    if(prepare_collect.seconds() > 0.65)
//                    {
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
//                        extendo_timer.reset();
//                        status = STROBOT.COLLECT_EXTENDO;
//                    }
//                    break;
//                }
//
//                case COLLECT_EXTENDO:
//                {
//                    if(park.seconds() < 25)
//                    {if(extendo_timer.seconds() > extendo_timer_i[nrcicluri])
//                    {
//                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//                        r.collect.setPower(1);
//
//                        switch (nrcicluri)
//                        {
//                            case 0:
//                            {
//                                collectAngle.collectAngle_i = 4;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//                            case 1:
//                            {
//                                collectAngle.collectAngle_i = 2;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//                            case 2:
//                            {
//                                collectAngle.collectAngle_i = 0;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//
//                        }
//                        status = STROBOT.COLLECT_VERIF_PIXLES;}
//
//                    }
//                    else
//                    {
//                        forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case COLLECT_VERIF_PIXLES:
//                {
//                    if(park.seconds() < 25)
//                    {if((!r.pixelLeft.getState() || !r.pixelRight.getState()))
//                    {
//                        failsafe2.reset();
//                        collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
//                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
//                    } else if(failsafe.seconds() > limit && tries <3 && (r.pixelLeft.getState() && r.pixelRight.getState()))
//                    {
//                       // redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.FAIL_SAFE;
//                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
//                        status = STROBOT.FAIL_SAFE;
//                    } else if (tries >=3)
//                    {
//                       // forced = true;
//                        r.collect.setPower(0);
//                        extendo.CS = extendoController.extendoStatus.RETRACTED;
//                        status = STROBOT.RETRACT_AND_RERTY;
//                    }
//
//                    }
//                    else
//                    { forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case RETRACT_AND_RERTY:
//                {
//                    if(park.seconds() < 25)
//                    { tries =0;
//                        funnypidtopoint.setTargetPose(retrypose);
//                    retry.reset();
//                    status = STROBOT.RETRY_TIMER_RESET;} else
//                    {
//                        forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case RETRY_TIMER_RESET:
//                {
//                    if(park.seconds() < 25)
//                    {  if(retry.seconds() > 1)
//                    {
//                        r.collect.setPower(1);
//                        switch (nrcicluri)
//                        {
//                            case 0:
//                            {
//                                collectAngle.collectAngle_i = 4;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//                            case 1:
//                            {
//                                collectAngle.collectAngle_i = 2;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//                            case 2:
//                            {
//                                collectAngle.collectAngle_i = 0;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//
//                        }
//                        status = STROBOT.COLLECT_VERIF_PIXLES;
//                    }
//                    } else
//                    {
//                        forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case FAIL_SAFE: {
//                    if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE)
//                    {
//                        limit = 0.6;
//                        tries += 1;
//                        failsafe.reset();
//                        status = STROBOT.COLLECT_VERIF_PIXLES;
//                    }
//                    break;
//                }
//
//                case FAIL_SAFE_HEADER_VERIF:
//                {
//                    if(park.seconds() < 25)
//                    {
//                        if(header.seconds() > 0.3 && (r.pixelLeft.getState() && r.pixelRight.getState()) &&  collectAngle.collectAngle_i >= 0)
//                        {
//                            extendo.CS = extendoController.extendoStatus.CYCLE;
//                            collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
//                            failsafe.reset();
//                            limit = 0.6;
//                            status = STROBOT.COLLECT_VERIF_PIXLES;
//
//                        } else if(header.seconds() > 0.3)
//                        {
//                            status = STROBOT.COLLECT_VERIF_PIXLES;
//                        }
//
//                    }
//                    else
//                    { forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//
//
//
//
//                case COLLECT_VERIF_PIXLES_V2:
//                {
//                    if(park.seconds() < 25)
//                    { if(!r.pixelLeft.getState() && !r.pixelRight.getState())
//                    {
//                        extendo.CS = extendoController.extendoStatus.RETRACTED;
//                        extendo_timer.reset();
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    } else if(failsafe2.seconds() > 0.6 && tries <3 && (r.pixelLeft.getState() || r.pixelRight.getState()))
//                    {                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
//
//                        status = STROBOT.FAIL_SAFE_2;
//                    } else if(tries >=3)
//                    {
//                        r.collect.setPower(0);
//                        extendo.CS = extendoController.extendoStatus.RETRACTED;
//                        status = STROBOT.RETRACT_AND_RERTY_v2;
//                    }
//
//                    }
//                    else
//                    { forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case RETRACT_AND_RERTY_v2:
//                {
//                    if(park.seconds() < 25)
//                    { tries =0;
//                        funnypidtopoint.setTargetPose(retrypose);
//                        retry.reset();
//                        status = STROBOT.RETRY_TIMER_RESET_v2;} else
//                    {
//                        forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case RETRY_TIMER_RESET_v2:
//                {
//                    if(park.seconds() < 25)
//                    {  if(retry.seconds() > 1)
//                    {
//                        r.collect.setPower(1);
//                        switch (nrcicluri)
//                        {
//                            case 0:
//                            {
//                                collectAngle.collectAngle_i = 3;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//                            case 1:
//                            {
//                                collectAngle.collectAngle_i = 1;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//                            case 2:
//                            {
//                                collectAngle.collectAngle_i = 0;
//                                extendo.CS = extendoController.extendoStatus.CYCLE;
//                                failsafe.reset();
//                                break;
//                            }
//
//                        }
//                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
//                    }
//                    } else
//                    {
//                        forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//
//                case FAIL_SAFE_2: {
//                    if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE)
//                    {
//                        failsafe2.reset();
//                        tries +=1;
//                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
//                    }
//                    break;
//                }
//
//                case FAIL_SAFE_HEADER_VERIF_2:
//                {
//                    if(park.seconds() < 25)
//                    {
//                        if(header.seconds() > 0.3 && (r.pixelLeft.getState() || r.pixelRight.getState()) &&  collectAngle.collectAngle_i >= 0)
//                        {
//                            extendo.CS = extendoController.extendoStatus.CYCLE;
//                          //  collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
//                            failsafe.reset();
//                            limit = 0.6;
//                            status = STROBOT.COLLECT_VERIF_PIXLES_V2;
//
//                        }else if(header.seconds() > 0.3)
//                        {
//                            status = STROBOT.COLLECT_VERIF_PIXLES_V2;
//                        }
//
//                    }
//                    else
//                    { forced = true;
//                        status = STROBOT.GO_SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//
//                case GO_SCORE_CYCLE:
//                {                        extendo.CS = extendoController.extendoStatus.RETRACTED;
//
//                    if(extendo_timer.seconds() > 0.2)
//                    { r.collect.setPower(0);}
//                    if(extendo_timer.seconds() > 0.3)
//                    {
//                        r.collect.setPower(-1);
//                        status = STROBOT.PREPARE_SCORE_CYCLE;
//                    }
//                    switch (nrcicluri) {
//                        case 0:
//                            switch (caz) {
//                                case 0:
//                                    funnypidtopoint.setTargetPose(score_second_cycle_center);
//                                    break;
//                                case 1:
//                                    funnypidtopoint.setTargetPose(score_second_cycle_center);
//                                    break;
//                                case 2:
//                                    funnypidtopoint.setTargetPose(score_second_cycle_center);
//                                    break;
//                            }
//                            break;
//
//                        case 1:
//                            switch (caz) {
//                                case 0:
//                                        funnypidtopoint.setTargetPose(score_second_cycle_center);
//                                    break;
//                                case 1:
//                                    funnypidtopoint.setTargetPose(score_second_cycle_center);
//                                    break;
//                                case 2:
//                                    funnypidtopoint.setTargetPose(score_second_cycle_center);
//                                    break;
//                            }
//                            break;
//
//
//                    }
//                    r.collect.setPower(-1);
//                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
//
//
//                    break;
//                }
//
//                case PREPARE_SCORE_CYCLE:
//                {r.collect.setPower(0);
//                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
//                    {
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
//                        nrcicluri +=1;
//                        score.reset();
//                        status = STROBOT.SCORE_CYCLE;
//                    }
//                    break;
//                }
//
//                case SCORE_CYCLE:
//                {
//                    if( redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_CYCLE_DONE && score.seconds() > 0.8)
//                    {
//                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;
//
//                        if(forced == false)
//                        {  switch (nrcicluri) {
//                            case 1:
//                                switch (caz) {
//                                    case 0:
//                                        funnypidtopoint.setTargetPoses(collect_third_cycle);
//                                        break;
//                                    case 1:
//                                        funnypidtopoint.setTargetPoses(collect_third_cycle);
//                                        break;
//                                    case 2:
//                                        funnypidtopoint.setTargetPoses(collect_third_cycle);
//                                        break;
//
//
//                                }
//                                break;
//
//                            default:
//                                funnypidtopoint.setTargetPose(parkpose);
//                                break;
//
//                        }
//                        prepare_collect.reset();
//                        if(nrcicluri <2)
//                        {  status= STROBOT.PREPARE_COLLECT;}
//                        else
//                        {  funnypidtopoint.setTargetPose(parkpose);
//                            park_systems.reset();
//                            status = STROBOT.PARK;
//                        }}
//                        else
//                        {  funnypidtopoint.setTargetPose(parkpose);
//                            status = STROBOT.PARK;
//                        }
//                    }
//
//                    break;
//                }
//
//                case PARK:
//                {
//                    if(park_systems.seconds() > 0.1)
//                    { redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
//                    status = STROBOT.NOTHING;}
//                    break;
//                }
//
//
//
//            }
//
//         //   funnypidtopoint.drive.update();
//            funnypidtopoint.update();
//            clawAngle.update(r);
//            clawFlip.update(r);
//            door.update(r);
//            fourbar.update(r);
//            latchLeft.update(r);
//            latchRight.update(r);
//            drone.update(r);
//            lift.update(r, position, voltage);
//            extendo.update(r, extendopos, 1, voltage);
//            collectAngle.update(r);
//            redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
//            failsafecontroller.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
//
//
//
////            telemetry.addData("status", status);
////            telemetry.addData("fourbar", fourbar.CS);
////            telemetry.addData("flip", clawFlip.CS);
////            telemetry.addData("angle", clawAngle.CS);
////            telemetry.addData("status autocontroller", redFarAutoController.CurrentStatus);
//
//            // telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
//            double loop = System.nanoTime();
//
//            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            telemetry.addData("status", status);
//            telemetry.addData("robotcontroller", RedFarAutoController.CurrentStatus);
//            telemetry.addData("poz", r.extendoLeft.getCurrentPosition());
//            telemetry.addData("extendo x", extendoController.x);
//            telemetry.addData("extendi", extendo.CS);
//            telemetry.addData("failsafe", org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.CurrentStatus);
//            telemetry.addData("tries", tries);
//            telemetry.addData("limit", limit);
//
//          //  telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.CM));
//            //  telemetry.addData("position", extendopos);
//            //   telemetry.addData("target", extendo.target);
//            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));
//
//            loopTime = loop;
//
//            telemetry.update();
//
//        }
//
//    }
//}
