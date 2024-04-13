package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe;
import org.firstinspires.ftc.teamcode.Auto.Recognition.BluePipelineStackMaster;
import org.firstinspires.ftc.teamcode.Auto.Recognition.RedPipelineStackMaster;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPixelMaster;

import org.firstinspires.ftc.teamcode.DistanceSensorCalibrator;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.globals.robotMap;

import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.droneController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import java.io.BufferedReader;
import java.util.List;
import java.util.logging.XMLFormatter;

@Photon
@Config
@Autonomous(group = "Auto" , name = "BlueFarMTI")

public class BlueFarMTI extends LinearOpMode {

    enum STROBOT {
        START,
        PURPLE,
        SYSTEMS_PURPLE,
        COLLECT_PURPLE,
        VERIF_TIMER,
        GO_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW,
        PREPARE_SCORE_YELLOW_v2,
        VERIF_PURPLE_SCORE,
        YELLOW_DROP,

        PREPARE_COLLECT,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXLES,
        COLLECT_VERIF_PIXLES_V2,

        GO_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE,
        SCORE_CYCLE,
        FAIL_SAFE,
       FAIL_SAFE_HEADER_VERIF,
        FAIL_SAFE_2,
        FAIL_SAFE_HEADER_VERIF_2,
        FAILSAFE_PURPLE,
        RETRACT_AND_RERTY,
        RETRY_TIMER_RESET,
        RETRACT_AND_RERTY_v2,
        RETRY_TIMER_RESET_v2,

        PARK,


        NOTHING
    }

    public static double x_start = -43, y_start = 61, angle_start = 90;

    /**
     * purple
     */

    public static double x_purple_preload_center = -54, y_purple_preload_center = 21, angle_purple_preload_center = 174;

    /**
     * yellow
     */

    public static double x_yellow_preload_center = 43, y_yellow_preload_center = 28, angle_yellow_preload_center = 180;


    /**
     * inter_purple
     */

    public static double inter_purple_x = -54, inter_purple_y = 33, inter_purple_angle = 180;

    /**
     * collect first stack
     */

    public static double x_collect_first_stack = -17, y_collect_first_stack = 33, angle_collect_first_stack = 175;

    /**
     * score first stack
     */

    public static double x_score_first_stack = 41.3,  y_score_first_stack = 29,  angle_score_first_stack = 180;



    /**
     * score second stack
     */

    public static double inter_score_second_score_x = 20, inter_score_second_score_y = 0, inter_score_second_score_angle = 177;
  //  public static double x_score_second_stack = 35,  y_score_second_stack = 33,  angle_score_second_stack = 177;

    public static double x_score_second_stack = 46.1, y_score_second_stack = 10, angle_score_second_stack = 205;

    /**
     * collect second stack
     */

    public static double inter_collect_second_score_x = 20, inter_collect_second_score_y = -2, inter_collect_second_score_angle = 180;
    public static double x_collect_second_stack = -21.5, y_collect_second_stack = 0, angle_collect_second_stack = 182;



    public static int caz = 0;
    public static double limit = 1.6;
    boolean forced = false;
    public static int tries = 0;
    public static int tries_purple = 0;
    public static int nrcicluri = 0;


    @Override
    public void runOpMode() throws InterruptedException {


        BluePipelineStackMaster blueRight = new BluePipelineStackMaster(this);
        blueRight.observeStick();

        DistanceSensorCalibrator calibrator;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotMap r = new robotMap(hardwareMap);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


        clawAngleController clawAngle = new clawAngleController();
        clawFlipController clawFlip = new clawFlipController();
        collectAngleController collectAngle = new collectAngleController();
        doorController door = new doorController();
        fourbarController fourbar = new fourbarController();
        latchLeftController latchLeft = new latchLeftController();
        latchRightController latchRight = new latchRightController();
        droneController drone = new droneController();
        liftController lift = new liftController();
        extendoController extendo = new extendoController();
        failsafe failsafecontroller = new failsafe();

        RedFarAutoController redFarAutoController = new RedFarAutoController();


        double voltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
        clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
        door.CS = doorController.doorStatus.CLOSED;
        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
        lift.CS = liftController.liftStatus.INITIALIZE;
        extendo.CS = extendoController.extendoStatus.RETRACTED;
        failsafecontroller.CurrentStatus = failsafe.failsafeStatus.NOTHING;
        // angle mai mare la al treilea ciclu
//
        drive.update();
        clawAngle.update(r);
        clawFlip.update(r);
        door.update(r);
        fourbar.update(r);
        latchLeft.update(r);
        latchRight.update(r);
        drone.update(r);
        lift.update(r, 0, voltage);
        extendo.update(r, 0, 1, voltage);
        redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
        failsafecontroller.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);

        collectAngle.update(r);


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        }

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));

        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));

        Pose2d inter_purple = new Pose2d(inter_purple_x, inter_purple_y, Math.toRadians(inter_purple_angle));

        Pose2d collect_first_cycle = new Pose2d(x_collect_first_stack+1, y_collect_first_stack, Math.toRadians(angle_collect_first_stack));
        Pose2d collect_first_cyclex = new Pose2d(x_collect_first_stack+1, y_collect_first_stack, Math.toRadians(angle_collect_first_stack+2));

        Pose2d score_first_cycle = new Pose2d(x_score_first_stack, y_score_first_stack, Math.toRadians(angle_score_first_stack));
        
        Pose2d score_second_cycle = new Pose2d(x_score_second_stack, y_score_second_stack, Math.toRadians(angle_score_second_stack));
        Pose2d inter_score_second_cycle = new Pose2d( inter_score_second_score_x ,  inter_score_second_score_y , Math.toRadians( inter_score_second_score_angle));

        Pose2d collect_second_cycle = new Pose2d(x_collect_second_stack, y_collect_second_stack, Math.toRadians(angle_collect_second_stack));
        Pose2d inter_collect_second_cycle = new Pose2d( inter_collect_second_score_x ,  inter_collect_second_score_y , Math.toRadians( inter_collect_second_score_angle));


        Pose2d inter_collect_second_cycle2 = new Pose2d( inter_collect_second_score_x ,  inter_collect_second_score_y - 2 , Math.toRadians( inter_collect_second_score_angle));
        Pose2d collect_second_cycle2 = new Pose2d(x_collect_second_stack, y_collect_second_stack, Math.toRadians(angle_collect_second_stack));


        Pose2d yellow = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(PURPLE_CENTER.end())
                .lineToLinearHeading(inter_purple)
                        .lineToLinearHeading(yellow)
                                .build();

        TrajectorySequence COLLECT_FIRST_CYCLE = drive.trajectorySequenceBuilder(YELLOW_CENTER.end())
                .lineToLinearHeading(collect_first_cycle)
                .build();

        TrajectorySequence COLLECT_FIRST_CYCLEx = drive.trajectorySequenceBuilder(YELLOW_CENTER.end())
                .lineToLinearHeading(collect_first_cyclex)
                .build();

        TrajectorySequence SCORE_FIRST_CYCLE = drive.trajectorySequenceBuilder(COLLECT_FIRST_CYCLE.end())
                        .lineToLinearHeading(score_first_cycle)
                                .build();


        TrajectorySequence COLLECT_SECOND_CYCLE = drive.trajectorySequenceBuilder(SCORE_FIRST_CYCLE.end())
                .setTangent(Math.toRadians(250))
                .splineToLinearHeading(new Pose2d(25, 8, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-19, 8, Math.toRadians(180)), Math.toRadians(180))
                .build();



        TrajectorySequence SCORE_SECOND_CYCLE = drive.trajectorySequenceBuilder(COLLECT_SECOND_CYCLE.end())
                .lineToLinearHeading(score_second_cycle)
                .build();

        TrajectorySequence COLLECT_SECOND_CYCLE2 = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE.end())
              //  .setTangent(Math.toRadians(250))
               // .splineToSplineHeading(new Pose2d(25, 8, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-20.5, 8, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence ParkBun = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE.end())
                .lineToLinearHeading(new Pose2d(40,  4, Math.toRadians(180)))
                .build();




        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;


        double loopTime = 0;
        double extendo_timer_i[] = {1, 1,1.8, 1.8, 1, 1};

        double score_timer_i[] = {0.405, 0.405,0.515, 0.515, 0.515, 0.65};

        double extendo_timer_i_purple[] = {1.8, 1.2, 1.4};

        ElapsedTime transfer = new ElapsedTime();
        ElapsedTime prepare_score_yellowqe = new ElapsedTime();
        ElapsedTime prepare_collect = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime verif_pixels = new ElapsedTime();
        ElapsedTime preload = new ElapsedTime();
        ElapsedTime verif = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime failsafe = new ElapsedTime();
        ElapsedTime failsafe2 = new ElapsedTime();
        ElapsedTime header = new ElapsedTime();
        ElapsedTime park = new ElapsedTime();
        ElapsedTime park_systems = new ElapsedTime();
        ElapsedTime failsafe_purple = new ElapsedTime();
        ElapsedTime retry = new ElapsedTime();

        extendo.caz = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;
        tries = 0;
        tries_purple =0;

        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(blueRight.opencvstack.getWhichSide() == "left"){
                caz = 0;
            } else if (blueRight.opencvstack.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", blueRight.opencvstack.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();
park.reset();
//        double[] rawReadings = {28.5, 27.3, 26.2, 24.9, 24.3, 23.6, 28.4, 29.5, 30.6, 31.2, 31.3, 31.6, 32.3, 33.4, 34.5, 35.3, 36.1, 37.8, 37.9, 38};
//        double[] actualDistances = {23, 22, 21.5, 21, 20.5, 20, 22.5, 24, 24.4, 24.6, 25.2, 25.5, 26, 26.6, 27.2, 28.1, 28.9, 29.6, 30, 30.5};
//        calibrator = new DistanceSensorCalibrator(rawReadings, actualDistances);



        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            int position = r.lift.getCurrentPosition();
            int extendopos = r.extendoLeft.getCurrentPosition();


            MotorConfigurationType motorConfigurationType = r.extendoRight.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoRight.setMotorType(motorConfigurationType);

            motorConfigurationType = r.extendoLeft.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoLeft.setMotorType(motorConfigurationType);

            motorConfigurationType = r.lift.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.lift.setMotorType(motorConfigurationType);

            //  extendo.distance= r.extendoDistance.getDistance(DistanceUnit.MM);

            switch (status) {

                case START: {

                    switch (caz)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            extendo.caz = 0;
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            extendo.caz = 1;
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            extendo.caz = 2;
                            break;
                        }
                    }

                    preload.reset();
                    status = STROBOT.SYSTEMS_PURPLE;
                    break;
                }

                case SYSTEMS_PURPLE:
                {
                    if(preload.seconds() > 0.15)
                    {
                        blueRight.stopCamera();
                        extendo_timer.reset();
                        status = STROBOT.PURPLE;
                    }
                    break;
                }

                case PURPLE:
                {
                    if(extendo_timer.seconds() > 0.85)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE;
                    }
                    if(extendo_timer.seconds() > extendo_timer_i_purple[caz])
                    {

                        r.collect.setPower(1);
                        preload.reset();
                        extendo.CS = extendoController.extendoStatus.PURPLE;
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROP;
                        status = STROBOT.COLLECT_PURPLE;
                    }
                    break;
                }

                case COLLECT_PURPLE:
                {
                    if((!r.pixelLeft.getState() && !r.pixelRight.getState()))
                    {

                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        verif.reset();
                        status = STROBOT.GO_SCORE_YELLOW;
                    }
                    break;

                }

                case FAILSAFE_PURPLE: {
                    if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE_PURPLE)
                    {
                        tries_purple += 1;
                        preload.reset();
                        status = STROBOT.COLLECT_PURPLE;
                    }
                    break;
                }

                case GO_SCORE_YELLOW:
                {

                    switch (caz)
                    {
                        case 0:

                        {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            if(verif.seconds() > 0.1)
                            {
                                r.collect.setPower(-1);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            if(verif.seconds() > 0.1)
                            {
                                r.collect.setPower(-1);
                                transfer.reset();
                                status = STROBOT.PREPARE_SCORE_YELLOW;
                            }
                           ;
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            if(verif.seconds() > 0.1)
                            {
                                r.collect.setPower(-1);
                            }
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                    }

                    break;

                }

                case PREPARE_SCORE_YELLOW:
                {

                    if(transfer.seconds() > 0.5)
                    {
                        r.collect.setPower(0);
                    }

                    if(transfer.seconds() > 1.2)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        prepare_score_yellowqe.reset();
                        status = STROBOT.PREPARE_SCORE_YELLOW_v2;
                    }
                    break;
                }

                case PREPARE_SCORE_YELLOW_v2:
                {
                    if( redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > -7)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN_BLUE;
                        score.reset();
                        status = STROBOT.VERIF_PURPLE_SCORE;
                    }

                    break;
                }

                case VERIF_PURPLE_SCORE:
                {
                    if( redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_YELLOW_DONE_BLUE && score.seconds() > 1.1)
                    {

                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP:
                {
//                    double rawReading = r.back.getDistance(DistanceUnit.CM);
//                    double calibratedDistance = calibrator.calibrate(rawReading);

                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;

                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE);
                                break;
                            }

                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE);
                                break;
                            }

                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_SECOND_CYCLE);
                                break;
                            }

                            case 3:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_SECOND_CYCLE2);
                                break;
                            }
                        }
                        prepare_collect.reset();
                        status= STROBOT.PREPARE_COLLECT;

                    break;
                }

                case PREPARE_COLLECT:
                {
                    if(prepare_collect.seconds() > 0.2)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        extendo_timer.reset();
                        status = STROBOT.COLLECT_EXTENDO;
                    }
                    break;
                }

                case COLLECT_EXTENDO:
                {
                    if(park.seconds() < 30)
                    {if(extendo_timer.seconds() > extendo_timer_i[nrcicluri])
                    {
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 3:
                            {
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 4:
                            {
                                collectAngle.collectAngle_i = 0;
                                extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXLES;}

                    }
                    else
                    {
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES:
                {
                   if((!r.pixelLeft.getState() || !r.pixelRight.getState()))
                    {
                        failsafe2.reset();
                        collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    }

                    break;
                }


                case COLLECT_VERIF_PIXLES_V2:
                {
                     if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        extendo_timer.reset();
                        status = STROBOT.GO_SCORE_CYCLE;
                    }

                    break;
                }



                case GO_SCORE_CYCLE:
                {
                    switch (nrcicluri) {
                    case 0:
                        drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                        break;

                    case 1:
                        drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                        break;

                    case 2:
                        drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE);
                        break;
                    case 3:
                        drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE);
                        break;


                }
                extendo.CS = extendoController.extendoStatus.RETRACTED;
                 //   r.collect.setPower(-1);
                    extendo_timer.reset();
                   // r.collect.setPower(-1);
                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                    status = STROBOT.PREPARE_SCORE_CYCLE;

                    break;
                }

                case PREPARE_SCORE_CYCLE:
                {
                    if(extendo_timer.seconds() > 0.25)
                    {r.collect.setPower(-1);}

                    if(extendo_timer.seconds() > 0.45)
                    {r.collect.setPower(0);}
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
                        nrcicluri +=1;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case SCORE_CYCLE:
                {
                    if( redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_CYCLE_DONE && score.seconds() >score_timer_i[nrcicluri])
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;

                        if(forced == false)
                        {  switch (nrcicluri) {

                            case 0:
                                        drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE);
                                        break;
                            case 1:
                                drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLEx);
                                break;
                            case 2:
                                drive.followTrajectorySequenceAsync(COLLECT_SECOND_CYCLE);
                                break;
                            case 3:
                                drive.followTrajectorySequenceAsync(COLLECT_SECOND_CYCLE2);
                                break;


                            default:
                                drive.followTrajectorySequenceAsync(ParkBun);
                                break;

                        }
                        prepare_collect.reset();
                        if(nrcicluri <=3)
                        {  status= STROBOT.PREPARE_COLLECT;}
                        else
                        { drive.followTrajectorySequenceAsync(ParkBun);
                            park_systems.reset();
                            status = STROBOT.PARK;
                        }}
                        else
                        { drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE);
                            status = STROBOT.PARK;
                        }
                    }

                    break;
                }

                case PARK:
                {
                    if(park_systems.seconds() > 0.15)
                    { redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                    status = STROBOT.NOTHING;}
                    break;
                }



            }

            drive.update();
            clawAngle.update(r);
            clawFlip.update(r);
            door.update(r);
            fourbar.update(r);
            latchLeft.update(r);
            latchRight.update(r);
            drone.update(r);
            lift.update(r, position, voltage);
            extendo.update(r, extendopos, 1, voltage);
            collectAngle.update(r);
            redFarAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
            failsafecontroller.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);



//            telemetry.addData("status", status);
//            telemetry.addData("fourbar", fourbar.CS);
//            telemetry.addData("flip", clawFlip.CS);
//            telemetry.addData("angle", clawAngle.CS);
//            telemetry.addData("status autocontroller", redFarAutoController.CurrentStatus);

            // telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            telemetry.addData("status", status);
//            telemetry.addData("robotcontroller", RedFarAutoController.CurrentStatus);
//            telemetry.addData("poz", r.extendoLeft.getCurrentPosition());
//            telemetry.addData("extendo x", extendoController.x);
//            telemetry.addData("extendi", extendo.CS);
//            telemetry.addData("failsafe", org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.CurrentStatus);
//            telemetry.addData("tries", tries);
//            telemetry.addData("limit", limit);
            telemetry.addData("nrcicluri", nrcicluri);

          //  telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.CM));
            //  telemetry.addData("position", extendopos);
            //   telemetry.addData("target", extendo.target);
            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}
