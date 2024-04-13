package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLOSE_DOOR;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DRIVE_POSE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_LATCHES;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_OPEN_DOOR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class transferController {

    public enum transferStatus
    {
        INITIALIZE,
        TRANSFER_EXTENDO,
        TRANSFER_OPEN_DOOR,
        TRANSFER_CLAW,
        TRANSFER_FOURBAR,
        TRANSFER_LATCHES,
        TRANSFER_DRIVE_POSE,
        TRANSFER_CLOSE_DOOR,
        TRANSFER_DONE,
        TRANSFER_CLOSE_DOORv2,
    }

    public transferController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    ElapsedTime door_timer = new ElapsedTime();
    ElapsedTime latches_timer = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();


    public static transferStatus CS = INITIALIZE, PS = INITIALIZE;

    public void update(robotMap r, doorController door, fourbarController fourbar, clawAngleController clawAngle, clawFlipController clawFlip, latchLeftController latchLeft, latchRightController latchRight, extendoController extendo, liftController lift)
    {
        if(CS != PS || CS == INITIALIZE || CS == TRANSFER_EXTENDO || CS == TRANSFER_OPEN_DOOR || CS == TRANSFER_FOURBAR || CS == TRANSFER_LATCHES || CS == TRANSFER_DRIVE_POSE || CS == TRANSFER_CLOSE_DOOR || CS== TRANSFER_CLAW || CS == TRANSFER_DONE)
        {
            switch (CS)
            {
                case TRANSFER_EXTENDO:
                {


                        extendo.CS = extendoController.extendoStatus.TRANSFER;
                        lift.CS = liftController.liftStatus.TRANSFER;


                    CS = TRANSFER_OPEN_DOOR;
                    break;
                }

                case TRANSFER_OPEN_DOOR:
                {
                    if(r.extendoLeft.getCurrentPosition() < 5 && r.collect.getPower() < 0.05 && r.collect.getPower() >-0.05)
                    {
                        door.CS = doorController.doorStatus.OPENED;
                        fourbar_timer.reset();
                        CS = TRANSFER_FOURBAR;
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
                        claw_timer.reset();
                        CS = TRANSFER_CLAW;}
                    break;
                }

                case TRANSFER_CLAW:
                {

                    clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT;
                    clawFlip.CS = clawFlipController.clawFlipStatus.COLLECT;
                    latches_timer.reset();
                    CS = TRANSFER_LATCHES;
                    break;
                }

                case TRANSFER_LATCHES:
                {
                    if(latches_timer.seconds() > 0.32)
                    {
                        latchLeft.CS = latchLeftController.LatchLeftStatus.SECURED;
                        latchRight.CS = latchRightController.LatchRightStatus.SECURED;
                        fourbar_timer.reset();
                        CS = TRANSFER_DRIVE_POSE;
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
                        door_timer.reset();
                        CS = TRANSFER_DONE;
                    }
                    break;

                }

                case TRANSFER_CLOSE_DOOR:
                {
                    if(door_timer.seconds() > 0.5)
                    {

                        CS = TRANSFER_DONE;
                    }
                    break;
                }
            }
        }

        PS = CS;

    }

}
