package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_BOTH;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_LEFT;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_LEFT1;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_LEFT2;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_RIGHT;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_RIGHT1;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_RIGHT2;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.INITIALIZE;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;


@Config
public class latchDropController {

    public enum latchDropStatus {
        INITIALIZE,
        DROP_ONE,
        DROP_ONE_RIGHT,
        DROP_ONE_RIGHT1,
        DROP_ONE_RIGHT2,
        DROP_ONE_LEFT,
        DROP_ONE_LEFT1,
        DROP_ONE_LEFT2,
        DROP_BOTH,
        DROP_DONE
    }

    public latchDropController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static latchDropStatus CS = INITIALIZE, PS = INITIALIZE;
    public boolean ok = FALSE;

    public void update(robotMap r, latchRightController latchRight, latchLeftController latchLeft, clawAngleController clawAngle) {
        if (CS != PS || CS == INITIALIZE || CS == DROP_ONE || CS == DROP_ONE_RIGHT1 || CS == DROP_ONE_LEFT1 || CS == DROP_ONE_RIGHT2 || CS == DROP_ONE_LEFT2 || CS == DROP_BOTH || CS == DROP_DONE || CS == DROP_ONE_LEFT || CS == DROP_ONE_RIGHT) {
            switch (CS) {

//                case DROP_ONE: {
//                    if (clawAngle.clawAngle_i <= 2) {
//                        if (ok == FALSE) {
//                            CS = DROP_ONE_LEFT1;
//                        } else {
//                            CS = DROP_ONE_LEFT2;
//                        }
//
//                    } else {
//                        if (ok == FALSE) {
//                            CS = DROP_ONE_RIGHT1;
//                        } else {
//                            CS = DROP_ONE_RIGHT2;
//                        }
//                    }
//                    break;
//                }

                case DROP_ONE_RIGHT:
                {
                    if (ok == FALSE) {
                        CS = DROP_ONE_RIGHT1;
                    } else {
                        CS = DROP_ONE_RIGHT2;
                    }
                    break;
                }

                case DROP_ONE_LEFT:
                {
                    if (ok == FALSE) {
                        CS = DROP_ONE_LEFT1;
                    } else {
                        CS = DROP_ONE_LEFT2;
                    }
                    break;
                }

                case DROP_ONE_RIGHT1: {

                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    ok = TRUE;
                    CS = DROP_DONE;
                    break;
                }
                case DROP_ONE_RIGHT2: {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    ok = FALSE;
                    CS = DROP_DONE;
                    break;
                }

                case DROP_ONE_LEFT1: {

                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    ok = TRUE;
                    CS = DROP_DONE;
                    break;
                }
                case DROP_ONE_LEFT2: {
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    ok = FALSE;
                    CS = DROP_DONE;
                    break;
                }

                case DROP_BOTH:
                {
                    latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    CS = DROP_DONE;
                    break;
                }
            }
        }

        PS = CS;
    }

}
