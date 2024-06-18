package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_LIFT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLLECT_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_LIFT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_LIFT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.robotMap;

import java.io.BufferedReader;

@Config
public class outtakeController {

    public enum outtakeStatus
    {
        INITIALIZE,
        SCORE_FOURBAR,
        SCORE_CLAW,
        SCORE_LIFT,
        SCORE_DONE,

        COLLECT_LIFT,
        COLLLECT_CLAW,
        COLLECT_FOURBAR,
        COLLECT_DONE,

        HANG_FOURBAR,
        HANG_LIFT,
        HANG_CLAW,
        HANG_DONE,

        HANG_DOWN,
    }

    public outtakeController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static outtakeStatus CS = INITIALIZE, PS = INITIALIZE;

    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime lift_timer = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();

    public void update(robotMap r, liftController lift, fourbarController fourbar, clawFlipController clawFlip, clawAngleController clawAngle, doorController door, latchRightController latchRight, latchLeftController latchLeft, transferController transfer)
    {
        if(CS != PS || CS == INITIALIZE || CS == SCORE_FOURBAR || CS == SCORE_LIFT || CS == SCORE_CLAW || CS == COLLECT_LIFT || CS == COLLLECT_CLAW || CS == COLLECT_FOURBAR || CS == SCORE_DONE || CS == COLLECT_DONE || CS == HANG_FOURBAR || CS == HANG_LIFT || CS == HANG_CLAW || CS == HANG_DONE)
        {
            switch (CS)
            {

                /**
                 * SCORE
                 */

                case SCORE_FOURBAR:
                {
                    if(transfer.CS == transferController.transferStatus.TRANSFER_DONE || transfer.CS == transferController.transferStatus.INITIALIZE || transfer.CS == transferController.transferStatus.TRANSFER_CLOSE_DOORv2)
                    {
                        fourbar.CS = fourbarController.fourbarStatus.SCORE;

                    lift_timer.reset();
                    CS = SCORE_LIFT;}
                    break;
                }

                case SCORE_LIFT:
                {
                    if(lift_timer.seconds() > 0.3)
                    {
                      //  transfer.CS = transferController.transferStatus.TRANSFER_CLOSE_DOORv2;
                        lift.pid = 1;
                        lift.CS = liftController.liftStatus.UP;
                        claw_timer.reset();
                        CS = SCORE_CLAW;
                    }
                    break;
                }

                case SCORE_CLAW:
                {
                    if(claw_timer.seconds() > 0.05)
                    {
                        clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                    }

                    if(claw_timer.seconds() > 0.1)
                    {
                        clawAngle.clawAngle_i = 2;
                        clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                        CS = SCORE_DONE;
                    }
                    break;
                }

                /**
                 * COLLECT
                 */
                case COLLECT_FOURBAR:
                {
                        fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                        claw_timer.reset();
                        CS = COLLLECT_CLAW;
                    break;
                }

                case COLLLECT_CLAW:
                {
                    if(claw_timer.seconds() > 0.1)
                    {
                        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                    }
                    if(claw_timer.seconds() > 0.2)
                    {
                        clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                        CS = COLLECT_LIFT;

                    } break;
                }

                case COLLECT_LIFT:
                {
                    door.CS = doorController.doorStatus.CLOSED;
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;

                    lift.pid = 0;
                    lift.CS = liftController.liftStatus.DOWN;
                    claw_timer.reset();
                    CS = COLLECT_DONE;
                    break;
                }

                /**
                 * HANG
                 */

                case HANG_FOURBAR:
                {
                    if(transfer.CS == transferController.transferStatus.TRANSFER_DONE || transfer.CS == transferController.transferStatus.INITIALIZE)
                    {  fourbar.CS = fourbarController.fourbarStatus.SCORE;
                        lift_timer.reset();
                        CS = HANG_LIFT;}
                    break;
                }

                case HANG_LIFT:
                {
                    if(lift_timer.seconds() > 0.3)
                    {
                        lift.pid = 1;
                        lift.CS = liftController.liftStatus.HANG;
                        claw_timer.reset();
                        CS = HANG_CLAW;
                    }
                    break;
                }

                case HANG_CLAW:
                {
                    if(claw_timer.seconds() > 0.05)
                    {
                        clawFlip.CS = clawFlipController.clawFlipStatus.SCORE;
                    }

                    if(claw_timer.seconds() > 0.1)
                    {
                        clawAngle.clawAngle_i = 2;
                        clawAngle.CS = clawAngleController.clawAngleStatus.SCORE;
                        CS = HANG_DONE;
                    }
                    break;
                }

                case HANG_DOWN:
                {
                    lift.CS = liftController.liftStatus.DOWN;
                    break;
                }


            }
        }

        PS = CS;
    }

}
