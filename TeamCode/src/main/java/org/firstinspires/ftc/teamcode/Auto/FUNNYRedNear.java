package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoControllers.RedFarAutoController;
import org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe;
import org.firstinspires.ftc.teamcode.Auto.Recognition.OpenCVMaster;

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


import java.util.ArrayList;
import java.util.List;

@Photon
@Config
@Autonomous(group = "Auto" , name = "FUNNYRedNear")

public class FUNNYRedNear extends LinearOpMode {

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

        GO_POWER_RANGERS,

        PREPARE_COLLECT,
        COLLECT_EXTENDO,
        COLLECT_VERIF_PIXLES,
        COLLECT_VERIF_PIXLES_V2,

        GO_SCORE_CYCLE,
        GO_SCORE_CYCLE_FUNNY_JAVA,
        GO_SCORE_CYCLE_FUNNY_JAVA_GROUND,
        PREPARE_SCORE_CYCLE,
        PREPARE_SCORE_CYCLE_ON_GROUND,
        SCORE_CYCLE,
        SCORE_CYCLE_GROUND,
        FAIL_SAFE,
        FAIL_SAFE_HEADER_VERIF,
        FAIL_SAFE_2,
        FAIL_SAFE_HEADER_VERIF_2,
        FAILSAFE_PURPLE,
        RETRACT_AND_RERTY,
        RETRY_TIMER_RESET,
        RETRACT_AND_RERTY_v2,
        RETRY_TIMER_RESET_v2,
        GO_COLLECT,
        GO_PARK,
        GO_SCORE_ON_GROUND,

        VERIF_FUNNY_MOMENT,
        FUNNY_MOMENT_CASE_ONE,
        FUNNY_MOMENT_CASE_TWO,

        VERIF_FUNNY_MOMENT_SCORE,
        FUNNY_MOMENT_CASE_ONE_SCORE,
        FUNNY_MOMENT_CASE_TWO_SCORE,

        PARK,


        NOTHING
    }

    public static double x_start = 16, y_start = -62, angle_start = 270;

    /**
     * purple
     */

    public static double x_purple_preload_right = 26, y_purple_preload_right = -45, angle_purple_preload_right = 270;
    public static double x_purple_preload_center = 7, y_purple_preload_center = -37, angle_purple_preload_center = 270;
    public static double x_purple_preload_left = 3, y_purple_preload_left = -40, angle_purple_preload_left = 290;

    /**
     * yellow
     */

    public static double x_yellow_preload_right = 47, y_yellow_preload_right = -44, angle_yellow_preload_right = 180;
    public static double x_yellow_preload_center = 47, y_yellow_preload_center = -35, angle_yellow_preload_center = 180;
    public static double x_yellow_preload_left = 47, y_yellow_preload_left = -28, angle_yellow_preload_left = 180;
    // public static double x_yellow_inter =

    /**
     * collect
     */



    // Cycle 2
    public static double x_inter_collect_cycle_2_right = 30, y_inter_collect_cycle_2_right = -7, angle_inter_collect_cycle_2_right = 180;
    public static double x_collect_cycle_2_right = -28, y_collect_cycle_2_right = -7, angle_collect_cycle_2_right = 180;

    public static double x_inter_collect_cycle_2_center = 30, y_inter_collect_cycle_2_center = -7, angle_inter_collect_cycle_2_center = 180;
    public static double x_collect_cycle_2_center = -30, y_collect_cycle_2_center = -7, angle_collect_cycle_2_center = 180;

    public static double x_inter_collect_cycle_2_left = 30, y_inter_collect_cycle_2_left = -7, angle_inter_collect_cycle_2_left = 180;
    public static double x_collect_cycle_2_left = -28, y_collect_cycle_2_left = -7, angle_collect_cycle_2_left = 180;

    // Cycle 3
    public static double x_inter_collect_cycle_3_right = 30, y_inter_collect_cycle_3_right = -7, angle_inter_collect_cycle_3_right = 180;
    public static double x_collect_cycle_3_right = -28, y_collect_cycle_3_right = -7, angle_collect_cycle_3_right = 180;

    public static double x_inter_collect_cycle_3_center = 30, y_inter_collect_cycle_3_center = -8, angle_inter_collect_cycle_3_center = 180;
    public static double x_collect_cycle_3_center = -30, y_collect_cycle_3_center = -8, angle_collect_cycle_3_center = 180;

    public static double x_inter_collect_cycle_3_left = 30, y_inter_collect_cycle_3_left = -7, angle_inter_collect_cycle_3_left = 180;
    public static double x_collect_cycle_3_left = -28, y_collect_cycle_3_left = -7, angle_collect_cycle_3_left = 180;

    public static double retry_x = -24.5, retry_y = 10, retry_angle = 180;

    /**
     * score
     */


    // Second cycle scoring positions
    public static double x_score_second_cycle_right = 50, y_score_second_cycle_right = -18, angle_score_second_angle_right = 150;
    public static double x_score_second_cycle_center = 50, y_score_second_cycle_center = -18, angle_score_second_angle_center = 150;
    public static double x_score_second_cycle_left = 50, y_score_second_cycle_left = -22.5, angle_score_second_angle_left = 150;

    // Third cycle scoring positions
    public static double x_score_third_cycle_right = 49.5, y_score_third_cycle_right = -16, angle_score_third_angle_right = 150;
    public static double x_score_third_cycle_center = 49.5, y_score_third_cycle_center = -16, angle_score_third_angle_center = 150;
    public static double x_score_third_cycle_left = 49.5, y_score_third_cycle_left = -21.5, angle_score_third_angle_left = 150;


    public static double x_score_forth_cycle_right = 49.5, y_score_forth_cycle_right = -16, angle_score_forth_angle_right = 150;
    public static double x_score_forth_cycle_center = 49.5, y_score_forth_cycle_center = -16, angle_score_forth_angle_center = 150;
    public static double x_score_forth_cycle_left = 49.5, y_score_forth_cycle_left = -21.5, angle_score_forth_angle_left = 150;





    /**
     * intern collect
     */

    public static double x_inter_collect_first_cycle = -44, y_inter_collect_first_cycle = -9, angle_inter_collect_first_cycle = 180;

    public static double x_inter_collect_first_cycle_left = -61, y_inter_collect_first_cycle_left = -9, angle_inter_collect_first_cycle_left = 180;

    public static double x_inter_collect_first_cycle_center = -57.5, y_inter_collect_first_cycle_center = -9, angle_inter_collect_first_cycle_center = 180;

    /**
     * intern score
     */

    public static double x_inter_score_first_cycle = 23, y_inter_score_first_cycle = -7, angle_inter_score_first_cycle =180;
    public static double x_inter_score_first_cycle_left = 23, y_inter_score_first_cycle_left = -7, angle_inter_score_first_cycle_left =180;

    public static int caz = 0;
    public static double limit = 2;
    boolean forced = false;
    public static int tries = 0;
    public static int tries_purple = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVMaster funnyRedNear = new OpenCVMaster(this);
        funnyRedNear.observeStick();

      //  DistanceSensorCalibrator calibrator;

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

        Pose2d purpleRight = new Pose2d(x_purple_preload_right, y_purple_preload_right, Math.toRadians(angle_purple_preload_right));
        Pose2d purpleCenter = new Pose2d(x_purple_preload_center, y_purple_preload_center, Math.toRadians(angle_purple_preload_center));
        Pose2d purpleLeft = new Pose2d(x_purple_preload_left, y_purple_preload_left, Math.toRadians(angle_purple_preload_left));

        Pose2d interCollectFirstCycle = new Pose2d(x_inter_collect_first_cycle, y_inter_collect_first_cycle, Math.toRadians(angle_inter_collect_first_cycle));
        Pose2d interScoreFirstCycle = new Pose2d(x_inter_score_first_cycle, y_inter_score_first_cycle, Math.toRadians(angle_inter_score_first_cycle));

        Pose2d interCollectFirstCycleleft = new Pose2d(x_inter_collect_first_cycle_left, y_inter_collect_first_cycle_left, Math.toRadians(angle_inter_collect_first_cycle_left));
        Pose2d interScoreFirstCycleleft = new Pose2d(x_inter_score_first_cycle_left, y_inter_score_first_cycle_left, Math.toRadians(angle_inter_score_first_cycle_left));

        Pose2d interCollectFirstCycleCenter = new Pose2d(x_inter_collect_first_cycle_center, y_inter_collect_first_cycle_center, Math.toRadians(angle_inter_collect_first_cycle_center));

        Pose2d yellowRight = new Pose2d(x_yellow_preload_right, y_yellow_preload_right, Math.toRadians(angle_yellow_preload_right));
        Pose2d yellowCenter = new Pose2d(x_yellow_preload_center, y_yellow_preload_center, Math.toRadians(angle_yellow_preload_center));
        Pose2d yellowLeft = new Pose2d(x_yellow_preload_left, y_yellow_preload_left, Math.toRadians(angle_yellow_preload_left));

        // Cycle 2 Right
        Pose2d collect_inter_cycle2_right = new Pose2d(x_inter_collect_cycle_2_right, y_inter_collect_cycle_2_right, Math.toRadians(angle_inter_collect_cycle_2_right));
        Pose2d collect_cycle2_right = new Pose2d(x_collect_cycle_2_right, y_collect_cycle_2_right, Math.toRadians(angle_collect_cycle_2_right));

// Cycle 2 Center
        Pose2d collect_inter_cycle2_center = new Pose2d(x_inter_collect_cycle_2_center, y_inter_collect_cycle_2_center, Math.toRadians(angle_inter_collect_cycle_2_center));
        Pose2d collect_cycle2_center = new Pose2d(x_collect_cycle_2_center, y_collect_cycle_2_center, Math.toRadians(angle_collect_cycle_2_center));

// Cycle 2 Left
        Pose2d collect_inter_cycle2_left = new Pose2d(x_inter_collect_cycle_2_left, y_inter_collect_cycle_2_left, Math.toRadians(angle_inter_collect_cycle_2_left));
        Pose2d collect_cycle2_left = new Pose2d(x_collect_cycle_2_left, y_collect_cycle_2_left, Math.toRadians(angle_collect_cycle_2_left));

// Cycle 3 Right
        Pose2d collect_inter_cycle3_right = new Pose2d(x_inter_collect_cycle_3_right, y_inter_collect_cycle_3_right, Math.toRadians(angle_inter_collect_cycle_3_right));
        Pose2d collect_cycle3_right = new Pose2d(x_collect_cycle_3_right, y_collect_cycle_3_right, Math.toRadians(angle_collect_cycle_3_right));

// Cycle 3 Center
        Pose2d collect_inter_cycle3_center = new Pose2d(x_inter_collect_cycle_3_center, y_inter_collect_cycle_3_center, Math.toRadians(angle_inter_collect_cycle_3_center));
        Pose2d collect_cycle3_center = new Pose2d(x_collect_cycle_3_center, y_collect_cycle_3_center, Math.toRadians(angle_collect_cycle_3_center));

// Cycle 3 Left
        Pose2d collect_inter_cycle3_left = new Pose2d(x_inter_collect_cycle_3_left, y_inter_collect_cycle_3_left, Math.toRadians(angle_inter_collect_cycle_3_left));
        Pose2d collect_cycle3_left = new Pose2d(x_collect_cycle_3_left, y_collect_cycle_3_left, Math.toRadians(angle_collect_cycle_3_left));



        // Second cycle scoring positions
        Pose2d score_second_cycle_right = new Pose2d(x_score_second_cycle_right, y_score_second_cycle_right, Math.toRadians(angle_score_second_angle_right));
        Pose2d score_second_cycle_center = new Pose2d(x_score_second_cycle_center, y_score_second_cycle_center, Math.toRadians(angle_score_second_angle_center));
        Pose2d score_second_cycle_left = new Pose2d(x_score_second_cycle_left, y_score_second_cycle_left, Math.toRadians(angle_score_second_angle_left));

// Third cycle scoring positions
        Pose2d score_third_cycle_right = new Pose2d(x_score_third_cycle_right, y_score_third_cycle_right, Math.toRadians(angle_score_third_angle_right));
        Pose2d score_third_cycle_center = new Pose2d(x_score_third_cycle_center, y_score_third_cycle_center, Math.toRadians(angle_score_third_angle_center));
        Pose2d score_third_cycle_left = new Pose2d(x_score_third_cycle_left, y_score_third_cycle_left, Math.toRadians(angle_score_third_angle_left));


        Pose2d score_forth_cycle_right = new Pose2d(x_score_forth_cycle_right, y_score_forth_cycle_right, Math.toRadians(angle_score_forth_angle_right));
        Pose2d score_forth_cycle_center = new Pose2d(x_score_forth_cycle_center, y_score_forth_cycle_center, Math.toRadians(angle_score_forth_angle_center));
        Pose2d score_forth_cycle_left = new Pose2d(x_score_forth_cycle_left, y_score_forth_cycle_left, Math.toRadians(angle_score_forth_angle_left));


        TrajectorySequence PURPLE_RIGHT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleRight)
                .build();

        TrajectorySequence PURPLE_CENTER = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleCenter)
                .build();

        TrajectorySequence PURPLE_LEFT = drive.trajectorySequenceBuilder(start_pose)
                .lineToLinearHeading(purpleLeft)
                .build();

        TrajectorySequence YELLOW_LEFT = drive.trajectorySequenceBuilder(purpleLeft)
                .lineToLinearHeading(yellowLeft)
//                .lineTo(new Vector2d(x_inter_score_first_cycle,y_inter_score_first_cycle))
//                .splineToConstantHeading(new Vector2d(x_yellow_preload_center,y_yellow_preload_center),Math.toRadians(0))
                .build();

        TrajectorySequence YELLOW_CENTER = drive.trajectorySequenceBuilder(purpleCenter)
                .lineToLinearHeading(yellowCenter)
//                .lineToLinearHeading(interCollectFirstCycleCenter)
//                .lineTo(new Vector2d(x_inter_score_first_cycle,y_inter_score_first_cycle))
//                .splineToConstantHeading(new Vector2d(x_yellow_preload_center,y_yellow_preload_center),Math.toRadians(0))
                .build();

        TrajectorySequence YELLOW_RIGHT = drive.trajectorySequenceBuilder(purpleRight)
                .lineToLinearHeading(yellowRight)
                .build();


        TrajectorySequence YELLOW_DROP_RIGHT = drive.trajectorySequenceBuilder(yellowRight)
                .back(12)
                .build();

        TrajectorySequence YELLOW_DROP_CENTER = drive.trajectorySequenceBuilder(yellowCenter)
                .back(12)
                .build();


        TrajectorySequence YELLOW_DROP_LEFT = drive.trajectorySequenceBuilder(yellowLeft)
                .back(12)
                .build();

        TrajectorySequence COLLECT_CYCLE_2_RIGHT = drive.trajectorySequenceBuilder(yellowRight)
                .setTangent(Math.toRadians(250))
                .splineToLinearHeading(new Pose2d(23, -59, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -59, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58.5, -37, Math.toRadians(162)), Math.toRadians(130))
                .build();

        TrajectorySequence COLLECT_CYCLE_2_CENTER = drive.trajectorySequenceBuilder(yellowCenter)
                .setTangent(Math.toRadians(250))
                .splineToLinearHeading(new Pose2d(23, -59, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -59, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58.5, -37, Math.toRadians(162)), Math.toRadians(130))
                .build();


        TrajectorySequence COLLECT_CYCLE_2_LEFT = drive.trajectorySequenceBuilder(yellowLeft)
                .setTangent(Math.toRadians(250))
                .splineToLinearHeading(new Pose2d(23, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58.5, -36, Math.toRadians(162)), Math.toRadians(130))
                .build();



        TrajectorySequence SCORE_SECOND_CYCLE_RIGHT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_RIGHT.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -37, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_CENTER = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_CENTER.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -37, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_SECOND_CYCLE_LEFT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_LEFT.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -37, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence COLLECT_CYCLE_3_RIGHT = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE_RIGHT.end())
                .setTangent(Math.toRadians(220))
                .splineToLinearHeading(new Pose2d(23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58.5, -37, Math.toRadians(162)), Math.toRadians(130))
                .build();

        TrajectorySequence COLLECT_CYCLE_3_CENTER = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE_CENTER.end())
                .setTangent(Math.toRadians(220))
                .splineToLinearHeading(new Pose2d(23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58.5, -37, Math.toRadians(162)), Math.toRadians(130))
                .build();

        TrajectorySequence COLLECT_CYCLE_3_LEFT = drive.trajectorySequenceBuilder(SCORE_SECOND_CYCLE_RIGHT.end())
                .setTangent(Math.toRadians(220))
                .splineToLinearHeading(new Pose2d(23, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58.5, -35, Math.toRadians(162)), Math.toRadians(130))
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_RIGHT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_RIGHT.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_CENTER = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_CENTER.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence SCORE_THIRD_CYCLE_LEFT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_LEFT.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence COLLECT_CYCLE_4_RIGHT = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_RIGHT.end())
                .setTangent(Math.toRadians(130))
                .splineToSplineHeading(new Pose2d(23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58, -33, Math.toRadians(175)), Math.toRadians(210))
                .build();

        TrajectorySequence COLLECT_CYCLE_4_CENTER = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_CENTER.end())
                .setTangent(Math.toRadians(130))
                .splineToSplineHeading(new Pose2d(23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58, -33, Math.toRadians(175)), Math.toRadians(210))
                .build();

        TrajectorySequence COLLECT_CYCLE_4_LEFT = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_LEFT.end())
                .setTangent(Math.toRadians(130))
                .splineToSplineHeading(new Pose2d(23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-23, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58, -33, Math.toRadians(175)), Math.toRadians(210))
                .build();


        TrajectorySequence SCORE_FORTH_CYCLE_RIGHT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_RIGHT.end())
                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(-23, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(49, -42, Math.toRadians(195)), Math.toRadians(320))
                .build();

        TrajectorySequence SCORE_FORTH_CYCLE_CENTER = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_CENTER.end())
                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(-23, -53, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(49, -42, Math.toRadians(195)), Math.toRadians(320))
                .build();

        TrajectorySequence SCORE_FORTH_CYCLE_LEFT = drive.trajectorySequenceBuilder(COLLECT_CYCLE_4_LEFT.end())
                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(-23, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(49, -42, Math.toRadians(195)), Math.toRadians(320))
                .build();

        TrajectorySequence FUNNY_MOMENT_ONE = drive.trajectorySequenceBuilder(new Pose2d(7, -55, Math.toRadians(180)))
                // .setTangent(Math.toRadians(130))
                .lineToLinearHeading(new Pose2d(23, -56, Math.toRadians(180)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-23, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58, -33, Math.toRadians(195)), Math.toRadians(210))
                .build();

        TrajectorySequence FUNNY_MOMENT_TWO = drive.trajectorySequenceBuilder(new Pose2d(-13, -55, Math.toRadians(180)))
                // .setTangent(Math.toRadians(130))
                .lineToLinearHeading(new Pose2d(-7, -56, Math.toRadians(180)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-23, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-58, -33, Math.toRadians(195)), Math.toRadians(210))
                .build();

        TrajectorySequence FUNNY_MOMENT_ONE_SCORE = drive.trajectorySequenceBuilder(new Pose2d(-16, -55, Math.toRadians(180)))
                // .setTangent(Math.toRadians(130))
                .lineToLinearHeading(new Pose2d(-23, -53, Math.toRadians(180)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(13, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(49, -42, Math.toRadians(165)), Math.toRadians(320))
                .build();

        TrajectorySequence FUNNY_MOMENT_TWO_SCORE = drive.trajectorySequenceBuilder(new Pose2d(-13, -55, Math.toRadians(180)))
                // .setTangent(Math.toRadians(130))
                .lineToLinearHeading(new Pose2d(-13, -53, Math.toRadians(180)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(13, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(49, -42, Math.toRadians(165)), Math.toRadians(320))
                .build();



        TrajectorySequence ParkBun = drive.trajectorySequenceBuilder(SCORE_THIRD_CYCLE_CENTER.end())
                .lineToLinearHeading(new Pose2d(42,  -60, Math.toRadians(180)))
                .build();

        TrajectorySequence ParkBun2 = drive.trajectorySequenceBuilder(COLLECT_CYCLE_3_CENTER.end())
                .setTangent(Math.toRadians(300))
                .splineToSplineHeading(new Pose2d(-23, -55, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(13, -55, Math.toRadians(180)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(42,  -60, Math.toRadians(180)))
                .build();

        TrajectorySequence RETRY = drive.trajectorySequenceBuilder(COLLECT_CYCLE_2_CENTER.end())
                .lineToLinearHeading(new Pose2d(-58, -32, Math.toRadians(185)))
                .build();


        drive.setPoseEstimate(start_pose);
        STROBOT status = STROBOT.START;

       // r.leftFront.setCurrentAlert(5, CurrentUnit.AMPS);

        int nrcicluri = 0;
        double loopTime = 0;
        double extendo_timer_i[] = {1.9, 1.75, 1.8, 1.8};

        double extendo_timer_i_purple[] = {0.95, 0.75, 1.9};
        double yellow_timer_i_purple[] = {0.95, 1.35, 1.9};

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
        ElapsedTime alilbitup = new ElapsedTime();
        ElapsedTime goscrcycl = new ElapsedTime();

        extendo.caz = 0;
        collectAngle.collectAngle_i = 4;
        lift.i_up = 0;
        tries = 0;
        tries_purple =0;
        List<Integer> constantsList = new ArrayList<>();
        constantsList.add(20);
        constantsList.add(20);
        constantsList.add(20);
        constantsList.add(20);

        while (!isStarted() && !isStopRequested()) {

            sleep(20);
            if(funnyRedNear.opencv.getWhichSide() == "left"){
                caz = 0;
            } else if (funnyRedNear.opencv.getWhichSide() == "center") {
                caz = 1;
            } else {
                caz = 2;
            }
            telemetry.addData("case", funnyRedNear.opencv.getWhichSide());
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
            int extendopos =r.extendoRight.getCurrentPosition();


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
                            drive.followTrajectorySequenceAsync(PURPLE_LEFT);
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
                            drive.followTrajectorySequenceAsync(PURPLE_RIGHT);
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
                    if(preload.seconds() > 0.169)
                    {
                        funnyRedNear.stopCamera();
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_funnyred;
                        extendo_timer.reset();
                        status = STROBOT.PURPLE;
                    }
                    break;
                }

                case PURPLE:
                {
                    if(!drive.isBusy())
                    {

                        preload.reset();
                        collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        if(caz == 1)
                        {  redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROPnearred;}
                        else
                        {
                            redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.PURPLE_DROPnearnucentrured;
                        }
                        status = STROBOT.GO_SCORE_YELLOW;
                    }
                    break;
                }



                case GO_SCORE_YELLOW:
                {
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.PURPLE_DROP_DONE )
                    { switch (caz)
                    {
                        case 0:

                        {

                            drive.followTrajectorySequenceAsync(YELLOW_LEFT);
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                        case 1:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_CENTER);
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;

                        }
                        case 2:
                        {
                            drive.followTrajectorySequenceAsync(YELLOW_RIGHT);
                            transfer.reset();
                            status = STROBOT.PREPARE_SCORE_YELLOW;
                            break;
                        }
                    }}

                    break;

                }

                case PREPARE_SCORE_YELLOW:
                {
                    if(transfer.seconds() > 0.15)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_YELLOW_BEGIN_BLUEnear;
                        preload.reset();
                        status = STROBOT.VERIF_PURPLE_SCORE;
                    }

                    break;
                }

                case VERIF_PURPLE_SCORE:
                {
                    if(!drive.isBusy())
                    {

                        status = STROBOT.YELLOW_DROP;
                    }
                    break;
                }

                case YELLOW_DROP:
                {
//                    double rawReading = r.back.getDistance(DistanceUnit.CM);
//                    double calibratedDistance = calibrator.calibrate(rawReading);

                    redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP_YELLOWnear;

                    status = STROBOT.GO_POWER_RANGERS;
                    break;
                }

                case GO_POWER_RANGERS:
                {
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.DIMA_O_SUGE_done)
                    {
                        switch (caz)
                        {
                            case 0:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_LEFT);
                                prepare_collect.reset();
                                status= STROBOT.PREPARE_COLLECT;
                                break;
                            }

                            case 1:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_CENTER);
                                prepare_collect.reset();
                                status= STROBOT.PREPARE_COLLECT;
                                break;
                            }

                            case 2:
                            {
                                drive.followTrajectorySequenceAsync(COLLECT_CYCLE_2_RIGHT);
                                prepare_collect.reset();
                                status= STROBOT.PREPARE_COLLECT;
                                break;
                            }
                        }
                    }
                    break;
                }

                case PREPARE_COLLECT:
                {
                    if(prepare_collect.seconds() > 0.3)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.COLLECT_PREPARE;
                        extendo_timer.reset();
                        status = STROBOT.COLLECT_EXTENDO;
                        r.collect.setPower(1);
                    }
                    break;
                }



                case COLLECT_EXTENDO:
                {
                    if(park.seconds() < 28)
                    {collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                //   extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                //  extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                //  extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 3:
                            {
                                collectAngle.collectAngle_i = 2;
                                // extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }



                        if(!drive.isBusy() || (!r.pixelLeft.getState() || !r.pixelRight.getState()))
                        {
                            status = STROBOT.COLLECT_VERIF_PIXLES;
                        }

                    }
                    else
                    {
                        forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_PARK;
                    }
                    break;
                }

                case COLLECT_VERIF_PIXLES:
                {
                    r.collect.setPower(1);
                    if(park.seconds() < 28)
                    {if((!r.pixelLeft.getState() || !r.pixelRight.getState()))
                    {
                        failsafe2.reset();
                        collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    } else if(failsafe.seconds() > limit && tries <3 && (r.pixelLeft.getState() && r.pixelRight.getState()))
                    {
                        // redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.FAIL_SAFE;
                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;
                        status = STROBOT.FAIL_SAFE;
                    } else if (tries >=3)
                    {
                        // forced = true;
                        r.collect.setPower(0);
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        status = STROBOT.RETRACT_AND_RERTY;
                    }

                    }
                    else
                    { forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_PARK;
                    }
                    break;
                }

                case RETRACT_AND_RERTY:
                {
                    if(park.seconds() < 28)
                    { tries =0;
                        drive.followTrajectorySequenceAsync(RETRY);
                        retry.reset();
                        status = STROBOT.RETRY_TIMER_RESET;} else
                    {
                        forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_SCORE_CYCLE;
                    }
                    break;
                }

                case RETRY_TIMER_RESET:
                {
                    if(park.seconds() < 28)
                    {  if(retry.seconds() > 1)
                    {
                        r.collect.setPower(1);
                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 4;
                                //  extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 2;
                                //  extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 0;
                                //   extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXLES;
                    }
                    } else
                    {
                        forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_PARK;
                    }
                    break;
                }

                case FAIL_SAFE: {
                    if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE)
                    {
                        limit = 1.2;
                        tries += 1;
                        failsafe.reset();
                        status = STROBOT.COLLECT_VERIF_PIXLES;
                    }
                    break;
                }

//                case FAIL_SAFE_HEADER_VERIF:
//                {
//                    if(park.seconds() < 28)
//                    {
//                        if(header.seconds() > 0.3 && (r.pixelLeft.getState() && r.pixelRight.getState()) &&  collectAngle.collectAngle_i >= 0)
//                        {
//                            extendo.CS = extendoController.extendoStatus.CYCLE;
//                            collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
//                            failsafe.reset();
//                            limit = 0.9;
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





                case COLLECT_VERIF_PIXLES_V2:
                {
                    if(park.seconds() < 28)
                    { if(!r.pixelLeft.getState() && !r.pixelRight.getState())
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        extendo_timer.reset();
                        goscrcycl.reset();
                        status = STROBOT.GO_SCORE_CYCLE;
                    } else if(failsafe2.seconds() > 1 && tries <3 && (r.pixelLeft.getState() || r.pixelRight.getState()))
                    {                        failsafecontroller.CurrentStatus = org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE;

                        status = STROBOT.FAIL_SAFE_2;
                    } else if(tries >=3)
                    {
                        r.collect.setPower(0);
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        status = STROBOT.RETRACT_AND_RERTY_v2;
                    }

                    }
                    else
                    { forced = true;
                        goscrcycl.reset();
                        status = STROBOT.GO_SCORE_ON_GROUND;
                    }
                    break;
                }

                case RETRACT_AND_RERTY_v2:
                {
                    if(park.seconds() < 28)
                    { tries =0;
                        drive.followTrajectorySequenceAsync(RETRY);
                        retry.reset();
                        status = STROBOT.RETRY_TIMER_RESET_v2;} else
                    {
                        goscrcycl.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_ON_GROUND;
                    }
                    break;
                }

                case RETRY_TIMER_RESET_v2:
                {
                    if(park.seconds() < 28)
                    {  if(retry.seconds() > 1)
                    {
                        r.collect.setPower(1);
                        switch (nrcicluri)
                        {
                            case 0:
                            {
                                collectAngle.collectAngle_i = 3;
                                //  extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 1:
                            {
                                collectAngle.collectAngle_i = 1;
                                //  extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }
                            case 2:
                            {
                                collectAngle.collectAngle_i = 4;
                                // extendo.CS = extendoController.extendoStatus.CYCLE;
                                failsafe.reset();
                                break;
                            }

                        }
                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    }
                    } else
                    {
                        goscrcycl.reset();
                        forced = true;
                        status = STROBOT.GO_SCORE_ON_GROUND;
                    }
                    break;
                }


                case FAIL_SAFE_2: {
                    if(failsafecontroller.CurrentStatus == org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.failsafeStatus.FAIL_SAFE_DONE)
                    {
                        failsafe2.reset();
                        tries +=1;
                        status = STROBOT.COLLECT_VERIF_PIXLES_V2;
                    }
                    break;
                }


                case GO_SCORE_CYCLE:
                {
                    if(goscrcycl.seconds() > 0.25)
                    {extendo.CS = extendoController.extendoStatus.RETRACTED;

                        switch (nrcicluri) {
                            case 0:
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(SCORE_SECOND_CYCLE_RIGHT);
                                        break;
                                }
                                break;

                            case 1:
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(SCORE_THIRD_CYCLE_RIGHT);
                                        break;
                                }
                                break;

                            case 2:
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_RIGHT);
                                        break;
                                }
                                break;

                            case 3:
                                switch (caz) {
                                    case 0:
                                        drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_LEFT);
                                        break;
                                    case 1:
                                        drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_CENTER);
                                        break;
                                    case 2:
                                        drive.followTrajectorySequenceAsync(SCORE_FORTH_CYCLE_RIGHT);
                                        break;
                                }
                                break;


                        }
                        extendo_timer.reset();
                        status = STROBOT.GO_SCORE_CYCLE_FUNNY_JAVA;}
                    //   r.collect.setPower(-1);
                    //  redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;


                    break;
                }

                case GO_PARK:
                {
                    extendo.CS = extendoController.extendoStatus.RETRACTED;
                    drive.followTrajectorySequenceAsync(ParkBun2);
                    status = STROBOT.NOTHING;
                    break;
                }

                case GO_SCORE_ON_GROUND:
                {
                    if(goscrcycl.seconds() > 0.25)
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        drive.followTrajectorySequenceAsync(ParkBun2);
                        status = STROBOT.GO_SCORE_CYCLE_FUNNY_JAVA_GROUND;
                    }
                    break;
                }

                case GO_SCORE_CYCLE_FUNNY_JAVA_GROUND:
                {
                    if(extendo_timer.seconds() > 0.35)
                    {
                        r.collect.setPower(-0.8);
                    }
                    if(extendo_timer.seconds() > 0.65)
                    {
                        r.collect.setPower(0);
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        status = STROBOT.PREPARE_SCORE_CYCLE_ON_GROUND;
                    }

                    break;
                }

                case GO_SCORE_CYCLE_FUNNY_JAVA:
                {
                    if(extendo_timer.seconds() > 0.45)
                    {
                        r.collect.setPower(-0.8);
                    }
                    if(extendo_timer.seconds() > 0.65)
                    {
                        r.collect.setPower(0);
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        status = STROBOT.PREPARE_SCORE_CYCLE;
                    }

                    break;
                }




                case PREPARE_SCORE_CYCLE:
                {r.collect.setPower(0);
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > 0)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_CYCLE_BEGIN;
                        nrcicluri +=1;
                        redFarAutoController.nr_cycle += 50;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }

                case PREPARE_SCORE_CYCLE_ON_GROUND:
                {r.collect.setPower(0);
                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.TRANSFER_DONE && drive.getPoseEstimate().getX() > 0)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.SCORE_ON_GROUND;
                        nrcicluri +=1;
                        score.reset();
                        status = STROBOT.SCORE_CYCLE_GROUND;
                    }
                    break;
                }

                case SCORE_CYCLE:
                {
                    if( redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.SCORE_CYCLE_DONE && score.seconds() > 1.1)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP;
                        status = STROBOT.GO_COLLECT;
                    }

                    break;
                }

                case SCORE_CYCLE_GROUND:
                {
                    if(score.seconds() > 0.95)
                    {
                        redFarAutoController.CurrentStatus = RedFarAutoController.autoControllerStatus.LATCH_DROP_YELLOW;
                        park_systems.reset();
                        status = STROBOT.PARK;
                    }

                    break;
                }

                case GO_COLLECT:
                {

                    if(redFarAutoController.CurrentStatus == RedFarAutoController.autoControllerStatus.NOTHING)
                    { if(forced == false)
                    {  switch (nrcicluri) {
                        case 1:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_3_RIGHT);
                                    break;
                            }
                            break;

                        case 2:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_RIGHT);
                                    break;
                            }
                            break;

                        case 3:
                            switch (caz) {
                                case 0:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_LEFT);
                                    break;
                                case 1:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_CENTER);
                                    break;
                                case 2:
                                    drive.followTrajectorySequenceAsync(COLLECT_CYCLE_4_RIGHT);
                                    break;
                            }
                            break;

                        default:
                            drive.followTrajectorySequenceAsync(ParkBun);
                            break;

                    }

                        if((nrcicluri <2 && park.seconds() <= 29) || nrcicluri<2 )
                        {  prepare_collect.reset();
                            status= STROBOT.PREPARE_COLLECT;}
                        else
                        { drive.followTrajectorySequenceAsync(ParkBun);
                            park_systems.reset();
                            status = STROBOT.PARK;
                        }}
                    else
                    { drive.followTrajectorySequenceAsync(ParkBun);
                        status = STROBOT.PARK;
                    }}
                    break;
                }

                case PARK:
                {
                    if(park_systems.seconds() > 0.5)
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
            telemetry.addData("status", status);
//            telemetry.addData("robotcontroller", RedFarAutoController.CurrentStatus);
//            telemetry.addData("extendo x", extendoController.x);
//            telemetry.addData("extendi", extendo.CS);
//            telemetry.addData("failsafe", org.firstinspires.ftc.teamcode.Auto.AutoControllers.failsafe.CurrentStatus);
//            telemetry.addData("tries", tries);
//            telemetry.addData("limit", limit);
//            telemetry.addData("targaet", lift.activePID.targetValue);

            //  telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.CM));
            //  telemetry.addData("position", extendopos);
            //   telemetry.addData("target", extendo.target);
            //    telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));

            loopTime = loop;

            telemetry.update();

        }

    }
}