package org.firstinspires.ftc.teamcode.Auto.AutoControllers;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.RedFar;

import org.firstinspires.ftc.teamcode.Auto.RedNear;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;


public class BlueNearAutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        PURPLE,
        PURPLE_DONE,

        PURPLE_DROP,
        PURPLE_DRIVE,
        PURPLE_DROP_DONE,
        PURPLE_DROPnear,

        TRANSFER_BEGIN,
        TRANSFER_FOURBAR,
        TRANSFER_CLAW,
        TRANSFER_LATCHES,
        TRANSFER_DRIVE_POSE,
        TRANSFER_DONE,

        SCORE_YELLOW_BEGIN,
        TIMER_RESET,
        SCORE_YELLOW_LIFT,
        SCORE_YELLOW_CLAW,
        SCORE_YELLOW_DONE,

        COLLECT_PREPARE,
        COLLECT_PREPARE_CLAW,
        COLLECT_PREPARE_LIFT,
        COLLECT_PREPARE_DONE,

        SCORE_CYCLE_BEGIN,
        TIMER_CYCLE_RESET,
        SCORE_CYCLE_LIFT,
        SCORE_CYCLE_CLAW,
        SCORE_CYCLE_DONE,

        FAIL_SAFE,
        FAIL_SAFE_CHECK_DISTANCE,
        FAIL_SAFE_HEADER,
        FAIL_SAFE_HEADER_TIMER_RESET,
        FAIL_SAFE_DONE,
        FAIL_SAFE_WRONG_HEADING,

        FAIL_SAFE_ONE_PIXEL,
        FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL,
        FAIL_SAFE_HEADER_ONE_PIXEL,
        FAIL_SAFE_HEADER_TIMER_RESET_ONE_PIXEL,
        FAIL_SAFE_DONE_ONE_PIXEL,
        FAIL_SAFE_WRONG_HEADING_ONE_PIXEL,
        FUNNY_JAVA,
        FUNNY_JAVA_ONE_PIXEL,

        LATCH_DROP,


    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    ElapsedTime purple_drive = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime latches_timer = new ElapsedTime();
    ElapsedTime pulamea = new ElapsedTime();
    ElapsedTime failsafe_header = new ElapsedTime();
    ElapsedTime funny_java = new ElapsedTime();

    double distance_error = 13;
    double correct_distance = 0;


    public void update(robotMap r, liftController lift, fourbarController fourbar, clawAngleController clawAngle, clawFlipController clawFlip, collectAngleController collectAngle, doorController door, extendoController extendo, latchLeftController latchLeft, latchRightController latchRight)
    {

       // double distance = r.extendoDistance.getDistance(DistanceUnit.CM);

        switch (CurrentStatus)
        {

            /**
             * PURPLE
             */

            case PURPLE:
            {

                fourbar.CS = fourbarController.fourbarStatus.PRELOAD;
                clawFlip.CS = clawFlipController.clawFlipStatus.PURPLE;
                CurrentStatus = autoControllerStatus.PURPLE_DONE;
                break;
            }
            case PURPLE_DROP:
            {
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE;
                break;
            }

            case PURPLE_DROPnear:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                purple_drive.reset();
                CurrentStatus = autoControllerStatus.PURPLE_DRIVE;
                break;
            }

            case PURPLE_DRIVE:
            {
                if(purple_drive.seconds() > 0.2)
                { fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                    clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                    CurrentStatus = autoControllerStatus.PURPLE_DROP_DONE;
                }
                break;
            }
            /**
             * YELLOW
             */

            case SCORE_YELLOW_BEGIN:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                CurrentStatus = autoControllerStatus.TIMER_RESET;
                break;
            }

            case TIMER_RESET:
            {
                pulamea.reset();
                CurrentStatus = autoControllerStatus.SCORE_YELLOW_LIFT;
                break;
            }

            case SCORE_YELLOW_LIFT:
            {
                if(pulamea.seconds() > 0.3)
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    lift.pid = 1;
                    lift.CS = liftController.liftStatus.YELLOW_NEAR;
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_CLAW;
                }
                break;
            }

            case SCORE_YELLOW_CLAW:
            {
                if(claw_timer.seconds() > 0.2)
                {
                    switch (RedNear.caz)
                    {
                        case 0:
                        { clawAngle.clawAngle_i = 3;
                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                            break;
                        }

                        case 1:
                        { clawAngle.clawAngle_i = 3;
                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                            break;
                        }

                        case 2:
                        {
                            clawAngle.clawAngle_i = 3;
                            clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                            break;
                        }
                    }
                    CurrentStatus = autoControllerStatus.SCORE_YELLOW_DONE;
                }
                break;
            }


            case LATCH_DROP:
            {
                latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                CurrentStatus = autoControllerStatus.NOTHING;
                break;
            }

            /**
             * PREPARECOLLECT
             */

            case COLLECT_PREPARE:
            {
                fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                claw_timer.reset();
                CurrentStatus = autoControllerStatus.COLLECT_PREPARE_CLAW;
                break;
            }

            case COLLECT_PREPARE_CLAW:
            {
                if(claw_timer.seconds() > 0.1)
                {
                    clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                }
                if(claw_timer.seconds() > 0.2)
                {
                    clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                    pulamea.reset();
                    CurrentStatus = autoControllerStatus.COLLECT_PREPARE_LIFT;
                }
                break;
            }

            case COLLECT_PREPARE_LIFT:
            {
                if(pulamea.seconds() > 0.2)
                {
                    lift.pid = 0;
                    lift.CS = liftController.liftStatus.DOWN;
                    CurrentStatus = autoControllerStatus.COLLECT_PREPARE_DONE;
                }
                break;
            }


            /**
             * TRANSFER
             */

            case TRANSFER_BEGIN:
            {
                extendo.CS = extendoController.extendoStatus.TRANSFER;
                lift.CS = liftController.liftStatus.TRANSFER;
                if(r.extendoLeft.getCurrentPosition() < 5)
                {
                    door.CS = doorController.doorStatus.OPENED;
                    fourbar_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_FOURBAR;
                }
                break;
            }

            case TRANSFER_FOURBAR:
            {
                if(fourbar_timer.seconds() > 0.3)
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    fourbar.CS = fourbarController.fourbarStatus.COLLECT;
                    CurrentStatus = autoControllerStatus.TRANSFER_CLAW;
                }
                break;
            }

            case TRANSFER_CLAW:
            {
                clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                latches_timer.reset();
                CurrentStatus = autoControllerStatus.TRANSFER_LATCHES;
                break;
            }

            case TRANSFER_LATCHES:
            {
                if(latches_timer.seconds() > 0.32)
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
                    latchRight.CS = latchRightController.LatchRightStatus.SECURED;
                    fourbar_timer.reset();
                    CurrentStatus =autoControllerStatus.TRANSFER_DRIVE_POSE;
                }
                break;
            }

            case TRANSFER_DRIVE_POSE:
            {
                if(fourbar_timer.seconds() > 0.2)
                {
                    lift.CS = liftController.liftStatus.DOWN;
                    extendo.CS = extendoController.extendoStatus.RETRACTED;
                    fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                }
                if(fourbar_timer.seconds() > 0.215)
                {
                    clawFlip.CS = clawFlipController.clawFlipStatus.DRIVE;
                }
                if(fourbar_timer.seconds() > 0.22)
                {
                    CurrentStatus = autoControllerStatus.TRANSFER_DONE;
                }
                break;

            }

            /**
             * Score Cycle
             */

            case SCORE_CYCLE_BEGIN:
            {
                fourbar.CS = fourbarController.fourbarStatus.SCORE;
                lift.pid = 1;
                lift.CS = liftController.liftStatus.CYCLE;
                clawAngle.clawAngle_i = 6;
                clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
               CurrentStatus = autoControllerStatus.SCORE_CYCLE_LIFT;
                break;
            }

            case SCORE_CYCLE_LIFT:
            {
                clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                door.CS = doorController.doorStatus.CLOSED;
                CurrentStatus = autoControllerStatus.SCORE_CYCLE_DONE;

                break;
            }

            /**
             * FAIL_SAFE
             */

            case FAIL_SAFE:
            {
                extendo.CS = extendoController.extendoStatus.FAIL_SAFE;
                failsafe_header.reset();
                CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER;
                break;
            }

//            case FAIL_SAFE_CHECK_DISTANCE:
//            {
//                if(distance > distance_error && (r.pixelRight.getState() && r.pixelLeft.getState()))
//                {
//                    extendo.x += 10;
//                    funny_java.reset();
//                    CurrentStatus = autoControllerStatus.FUNNY_JAVA;
//                }
//                else
//                {
//                    if(!r.pixelRight.getState() || !r.pixelLeft.getState())
//                    {
//
//                        //  extendo.cycle = extendo.failsafe + extendo.x;
//                        //  extendo.CS = extendoController.extendoStatus.CYCLE;
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE;
//                    }
//                    else
//                    {
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER;
//                    }
//                }
//                break;
//            }
//
//            case FUNNY_JAVA:
//            {
//                if(funny_java.seconds() > 0.01)
//                {
//                    CurrentStatus = autoControllerStatus.FAIL_SAFE_CHECK_DISTANCE;
//                }
//                break;
//            }

            case FAIL_SAFE_HEADER:
            {
                if(failsafe_header.seconds() > 0.2 && collectAngle.collectAngle_i >= 1)
                {
                    collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                    collectAngle.collectAngle_i -= 1;
                    extendo.CS = extendoController.extendoStatus.CYCLE;
                    funny_java.reset();
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE;
                }
                break;
            }

            case FAIL_SAFE_HEADER_TIMER_RESET:
            {
                if(failsafe_header.seconds() > 0.01)
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER;
                }
                break;
            }


//            case FAIL_SAFE_ONE_PIXEL:
//            {
//                if(distance > distance_error)
//                {
//                    extendo.CS = extendoController.extendoStatus.FAIL_SAFE;
//                    CurrentStatus = autoControllerStatus.FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL;
//                } else
//                {
//                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_ONE_PIXEL;
//                }
//                break;
//
//            }
//
//            case FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL:
//            {
//                if(distance > distance_error && (r.pixelRight.getState() || r.pixelLeft.getState()))
//                {
//                    extendo.x += 20;
//                    funny_java.reset();
//                    CurrentStatus = autoControllerStatus.FUNNY_JAVA_ONE_PIXEL;
//                }
//                else
//                {
//                    if(!r.pixelRight.getState() && !r.pixelLeft.getState())
//                    {
//
//                      //  extendo.cycle = extendo.failsafe + extendo.x;
//                       // extendo.CS = extendoController.extendoStatus.CYCLE;
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE_ONE_PIXEL;
//                    }
//                    else
//                    {
//                        CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_ONE_PIXEL;
//                    }
//                }
//                break;
//            }

            case FUNNY_JAVA_ONE_PIXEL:
            {
                if(funny_java.seconds() >0.01)
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_CHECK_DISTANCE_ONE_PIXEL;
                }
                break;
            }

            case FAIL_SAFE_HEADER_ONE_PIXEL:
            {
                if((r.pixelRight.getState() || r.pixelLeft.getState()) && collectAngle.collectAngle_i != 0)
                {
                    collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
                    failsafe_header.reset();
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_TIMER_RESET_ONE_PIXEL;
                } else if(!r.pixelRight.getState() && !r.pixelLeft.getState())
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_DONE_ONE_PIXEL;
                } else
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_WRONG_HEADING_ONE_PIXEL;
                }

                break;
            }

            case FAIL_SAFE_HEADER_TIMER_RESET_ONE_PIXEL:
            {
                if(failsafe_header.seconds() > 0.5)
                {
                    CurrentStatus = autoControllerStatus.FAIL_SAFE_HEADER_ONE_PIXEL;
                }
                break;
            }




        }
        PreviousStatus = CurrentStatus;
    }
}

