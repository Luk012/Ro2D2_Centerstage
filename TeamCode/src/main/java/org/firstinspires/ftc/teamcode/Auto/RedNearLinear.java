package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoControllers.BlueNearAutoController;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe;
import org.firstinspires.ftc.teamcode.Auto.Recognition.OpenCVMaster;
import org.firstinspires.ftc.teamcode.Auto.Recognition.RedPipelineStackMaster;
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

import java.util.List;
import java.util.Vector;

@Config
@Autonomous(group = "Auto" , name = "RedNearLinear")

public class RedNearLinear extends LinearOpMode {

    enum STROBOT {
        VERIF_CASE,
        START,
        PURPLE,
        GO_SCORE_YELLOW,
        YELLOW_DROP,
        GO_COLLECT,
        CHECK_GO_COLLECT,
        COLLECT,
        COLLECT_PREPARE,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXELS,
        COLLECT_VERIF_PIXELS_V2,
        GO_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE,
        SCORE_CYCLE,
        FAIL_SAFE,
        FAIL_SAFE_WRONG_HEADING,
        FAIL_SAFE_ONE_PIXEL,
        FAIL_SAFE_WRONG_HEADING_ONE_PIXEL,
        RETRACT_AND_RERTY,
        RETRY_TIMER_RESET,
        FAIL_SAFE_2,
        PARK,

        NOTHING
    }

    public static double x_start = 16, y_start = -62, angle_start = 270;
    public static int tries;
    public static double limit;


    /**
     * purple
     */

    public static double x_purple_preload_right = 25, y_purple_preload_right = -40, angle_purple_preload_right = 270;
    public static double x_purple_preload_center = 16, y_purple_preload_center = -37.5, angle_purple_preload_center = 270;
    public static double x_purple_preload_left = 13.5, y_purple_preload_left = -32, angle_purple_preload_left = 320;

    /**
     * yellow
     */

    public static double x_yellow_preload_right = 47.5, y_yellow_preload_right = -36, angle_yellow_preload_right = 180;
    public static double x_yellow_preload_center = 47.5, y_yellow_preload_center = -28, angle_yellow_preload_center = 180;
    public static double x_yellow_preload_left = 47.5, y_yellow_preload_left = -21, angle_yellow_preload_left = 180;


    /**
     * first cycle collect
     */

    public static double x_inter_backdrop_first_cycle = 25, y_inter_backdrop_first_cycle = -58;
    public static double x_change_heading_first_cycle = -12, y_change_heading_first_cycle = -58;
    public static double x_collect_first_cycle = -24, y_collect_first_cycle = -58, angle_collect_first_cycle = 155;

    /**
     * first cycle score
     */

    public static double x_change_heading_first_cycle_score = 0, y_change_heading_first_cycle_score = -58, angle_change_heading_first_cycle_score = 180;
    public static double x_inter_backdrop_first_cycle_score = 35, y_inter_backdrop_first_cycle_score = -58;
    public static double x_score_first_cycle = 50, y_score_first_cycle = -36;


    public static int caz = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVMaster redNear = new OpenCVMaster(this);
        redNear.observeStick();




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

        BlueNearAutoController blueNearAutoController = new BlueNearAutoController();


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
        drone.CS = droneController.droneStatus.SECURED;
        failsafecontroller.CurrentStatus = failsafe.failsafeStatus.NOTHING;
        failsafecontroller.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);

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
        blueNearAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);

        collectAngle.update(r);



        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        }

        Pose2d start_pose = new Pose2d(x_start, y_start,Math.toRadians(angle_start));

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));

        Pose2d yellowRight = new Pose2d(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
        Pose2d yellowCenter = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
        Pose2d yellowLeft = new Pose2d(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));

        Pose2d inter_backdrop_first_cycle = new Pose2d(x_inter_backdrop_first_cycle, y_inter_backdrop_first_cycle, Math.toRadians(180));

        Pose2d inter_backdrop_first_cycle2 = new Pose2d(x_inter_backdrop_first_cycle, y_inter_backdrop_first_cycle-2, Math.toRadians(180));
        Pose2d change_heading_first_cycle = new Pose2d(x_change_heading_first_cycle, y_change_heading_first_cycle,Math.toRadians(180));

        Pose2d collect_first_cycle = new Pose2d(x_collect_first_cycle,y_collect_first_cycle, Math.toRadians(angle_collect_first_cycle));
        Pose2d collect_first_cycle2 = new Pose2d(x_collect_first_cycle,y_collect_first_cycle-4, Math.toRadians(angle_collect_first_cycle));

        Pose2d collect_first_cycle_right = new Pose2d(x_collect_first_cycle-4,y_collect_first_cycle, Math.toRadians(angle_collect_first_cycle));


        Pose2d change_heading_first_cycle_score = new Pose2d(x_change_heading_first_cycle_score, y_change_heading_first_cycle_score, Math.toRadians(angle_change_heading_first_cycle_score));
        Pose2d inter_backdrop_first_cycle_score= new Pose2d(x_inter_backdrop_first_cycle_score, y_inter_backdrop_first_cycle_score,Math.toRadians(180));
        Pose2d score_first_cycle = new Pose2d(x_score_first_cycle, y_score_first_cycle,Math.toRadians(180));
        // First cycle poses

        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                //   .lineToLinearHeading(inter_yellow)
                .lineToLinearHeading(purpleLeft)
                .build();

        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purpleLeft)
                .lineToLinearHeading(new Pose2d(24, -50, Math.toRadians(270)))
                //  .lineToLinearHeading(inter_yellow)
                .lineToLinearHeading(yellowLeft)
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purpleCenter)
                //    .lineToLinearHeading(inter_yellow)
                .lineToLinearHeading(yellowCenter)
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(new Pose2d(24, -50, Math.toRadians(270)))
                .lineToLinearHeading(yellowRight)
                .build();

        TrajectorySequence COLLECT_FIRST_CYCLE_LEFT = drive.trajectorySequenceBuilder(YELLOW_LEFT.end())
                .lineToLinearHeading(inter_backdrop_first_cycle)
                .lineToLinearHeading(change_heading_first_cycle)
                .lineToLinearHeading(collect_first_cycle)
                .build();

        TrajectorySequence COLLECT_FIRST_CYCLE_CENTER = drive.trajectorySequenceBuilder(YELLOW_CENTER.end())
                .lineToLinearHeading(inter_backdrop_first_cycle)
                .lineToLinearHeading(change_heading_first_cycle)
                .lineToLinearHeading(collect_first_cycle)
                .build();

        TrajectorySequence COLLECT_FIRST_CYCLE_RIGHT = drive.trajectorySequenceBuilder(YELLOW_RIGHT.end())
                .lineToLinearHeading(inter_backdrop_first_cycle)
                .lineToLinearHeading(change_heading_first_cycle)
                .lineToLinearHeading(collect_first_cycle)
                .build();

        TrajectorySequence SCORE_FIRST_CYCLE = drive.trajectorySequenceBuilder(COLLECT_FIRST_CYCLE_RIGHT.end())
                .lineToLinearHeading(change_heading_first_cycle)
                .lineToLinearHeading(inter_backdrop_first_cycle_score)
                .lineToLinearHeading(score_first_cycle)
                .build();

        TrajectorySequence COLLECT_SECOND_CYCLE = drive.trajectorySequenceBuilder(SCORE_FIRST_CYCLE.end())
                .lineToLinearHeading(inter_backdrop_first_cycle2)
                .lineToLinearHeading(change_heading_first_cycle)
                .lineToLinearHeading(collect_first_cycle2)
                .build();

        TrajectorySequence ParkBun = drive.trajectorySequenceBuilder(SCORE_FIRST_CYCLE.end())
                .lineToLinearHeading(new Pose2d(45,  -58, Math.toRadians(180)))
                .build();





        // First cycle trajectories


        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.VERIF_CASE;
        tries = 0;

        int nrcicluri = 0;
        double loopTime = 0;
        ElapsedTime purple_timer = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime prepare_collect = new ElapsedTime();
        ElapsedTime score = new ElapsedTime();
        ElapsedTime failsafe = new ElapsedTime();
        ElapsedTime failsafe2 = new ElapsedTime();
        ElapsedTime park = new ElapsedTime();
        ElapsedTime retry =new ElapsedTime();
        ElapsedTime park_systems = new ElapsedTime();
        ElapsedTime score_yellow = new ElapsedTime();
        ElapsedTime verif_case = new ElapsedTime();
        ElapsedTime scuipa = new ElapsedTime();

        extendo.caz = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;
        limit = 1.6;
        boolean forced = false;


        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(redNear.opencv.getWhichSide() == "left"){
                caz = 0;
            } else if (redNear.opencv.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", redNear.opencv.getWhichSide());
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        park.reset();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            int position = r.lift.getCurrentPosition();
            int extendopos = r.extendoRight.getCurrentPosition();


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

                case VERIF_CASE:
                {
                    verif_case.reset();
                    status= STROBOT.START;
                    break;
                }

                case START: {

                    if (caz == 0 && verif_case.seconds() > 5)
                    {switch (caz)
                    {
                        case 0:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                            break;
                        }
                    }


                    purple_timer.reset();
                    blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.PURPLE;
                    status = STROBOT.PURPLE;}
                    else if(caz != 0)
                    {
                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(PURPLE_LEFT);
                                break;
                            }
                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(PURPLE_CENTER);
                                break;

                            }
                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
                                break;
                            }
                        }

                        purple_timer.reset();
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.PURPLE;
                        status = STROBOT.PURPLE;
                    }
                    break;
                }

                case PURPLE:
                {
                    if(!drive.isBusy())
                    {
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.PURPLE_DROP;
                        status = STROBOT.GO_SCORE_YELLOW;
                    }
                    break;
                }

                case GO_SCORE_YELLOW:
                {
                    if(blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.PURPLE_DROP_DONE)
                    {
                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                                break;
                            }
                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                                break;

                            }
                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                                break;
                            }
                        }
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN;
                        status= STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP:
                {
                    if(!drive.isBusy() && blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.SCORE_YELLOW_DONE)
                    {


                        status = STROBOT.CHECK_GO_COLLECT;
                    }
                    break;
                }

                case CHECK_GO_COLLECT:
                {
                    BlueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.LATCH_DROP;
                    score_yellow.reset();
                    status = STROBOT.GO_COLLECT;

                    break;
                }

                case GO_COLLECT:
                {
                    if(score_yellow.seconds() > 0.15)
                    {  switch (nrcicluri) {
                        case 0: // Assuming this is for the first cycle
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(COLLECT_FIRST_CYCLE_RIGHT);
                                    break;
                            }
                            break;


                        default:
                            // Handle unexpected number of cycles
                            System.out.println("Unexpected number of cycles: " + nrcicluri);
                    }
                        prepare_collect.reset();
                        status = STROBOT.COLLECT_PREPARE;}
                    break;
                }

                case COLLECT_PREPARE:
                {
                    if(prepare_collect.seconds() > 0.4)
                    { blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.COLLECT_PREPARE;
                        status= STROBOT.COLLECT_EXTENDO;}
                    break;
                }

                case COLLECT_EXTENDO:
                {
                    if(park.seconds()<20.5)
                    {if(!drive.isBusy())
                    {
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        r.collect.setPower(1);

                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                extendo.CS = extendoController.extendoStatus.EXTENDED_NEAR;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                extendo.CS = extendoController.extendoStatus.EXTENDED_NEAR;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                extendo.CS = extendoController.extendoStatus.EXTENDED_NEAR;
                                failsafe.reset();
                                break;
                            }

                        }
                        limit = 2;
                        status = STROBOT.COLLECT_VERIF_PIXELS;
                    }}
                    else{
                        scuipa.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXELS:
                {
                    if(park.seconds() < 20.5)
                    {if((!r.pixelLeft.getState() || !r.pixelRight.getState()))
                    {
                        failsafe2.reset();
                        collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
                        status = STROBOT.COLLECT_VERIF_PIXELS_V2;
                    } else if(failsafe.seconds() > limit && tries <3 && (r.pixelLeft.getState() && r.pixelRight.getState()))
                    {
                        // redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.FAIL_SAFE;
                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_NEAR;
                        status = STROBOT.FAIL_SAFE;
                    } else if (tries >=3)
                    {scuipa.reset();
                        // forced = true;
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }

                    }
                    else
                    { scuipa.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }


                case FAIL_SAFE:
                {
                    if(park.seconds() <20.5)
                    { if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE_NEAR)
                    {
                        limit = 0.8;
                        tries += 1;
                        failsafe.reset();
                        status = STROBOT.COLLECT_VERIF_PIXELS;
                    }}
                    else
                    {
                        scuipa.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }



                case COLLECT_VERIF_PIXELS_V2:
                {
                    if(park.seconds() < 20.5)
                    { if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        extendo_timer.reset();
                        scuipa.reset();
                        status = STROBOT.GO_SCORE_CYCLE;
                    } else if(failsafe2.seconds() > 0.6 && tries <3 && (r.pixelLeft.getState() || r.pixelRight.getState()))
                    {                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;

                        status = STROBOT.FAIL_SAFE_2;
                    } else if(tries >=3)
                    {scuipa.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }

                    }
                    else
                    {scuipa.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }

                case FAIL_SAFE_2: {
                    if(park.seconds() < 20.5)
                    {if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE)
                    {
                        failsafe2.reset();
                        tries +=1;
                        status = STROBOT.COLLECT_VERIF_PIXELS_V2;
                    }}
                    else
                    {
                        scuipa.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }


                case GO_SCORE_CYCLE:
                {extendo.CS = extendoController.extendoStatus.RETRACTED;

                    switch (nrcicluri) {
                        case 0:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                            }
                            break;

                        case 1:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                            }
                            break;
                        case 2:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(SCORE_FIRST_CYCLE);
                                    break;
                            }
                            break;

                    }
                    if(scuipa.seconds() > 0.1)
                    {
                        r.collect.setPower(-1);
                    }
                    if(scuipa.seconds() > 0.3)
                    { r.collect.setPower(0);
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.TRANSFER_BEGIN;
                    status = STROBOT.PREPARE_SCORE_CYCLE;}
                    break;
                }

                case PREPARE_SCORE_CYCLE:
                {r.collect.setPower(0);
                    if(blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > 10)
                    {
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
                        nrcicluri +=1;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case SCORE_CYCLE:
                {
                    if( blueNearAutoController.CurrentStatus == BlueNearAutoController.autoControllerStatus.SCORE_CYCLE_DONE && !drive.isBusy())
                    {
                        blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.LATCH_DROP;
                        park_systems.reset();
                        drive.followTrajectorySequenceAsync(ParkBun);
                            status = STROBOT.PARK;

                    }

                    break;
                }

                case PARK:
                {
                    if(park_systems.seconds() > 0.4)
                    { blueNearAutoController.CurrentStatus = BlueNearAutoController.autoControllerStatus.COLLECT_PREPARE;
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
            blueNearAutoController.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);
            failsafecontroller.update(r, lift, fourbar,clawAngle,clawFlip,collectAngle,door,extendo,latchLeft,latchRight);




//            telemetry.addData("status", status);
//            telemetry.addData("fourbar", fourbar.CS);
//            telemetry.addData("flip", clawFlip.CS);
//            telemetry.addData("angle", clawAngle.CS);
//            telemetry.addData("status autocontroller", redFarAutoController.CurrentStatus);

            // telemetry.addData("distance", r.back.getDistance(DistanceUnit.CM));
            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("status", status);
            telemetry.addData("robotcontroller", blueNearAutoController.CurrentStatus);
            //  telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("x", extendo.x);
            telemetry.addData("CS", lift.CS);
            telemetry.addData("liftcp", lift.CurrentPosition);
            telemetry.addData("extendocp", r.extendoLeft.getCurrentPosition());
            //  telemetry.addData("position", extendopos);
            //   telemetry.addData("target", extendo.target);
            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}
