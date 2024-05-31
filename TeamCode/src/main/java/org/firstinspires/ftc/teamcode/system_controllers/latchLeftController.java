package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.latchLeftController.LatchLeftStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class latchLeftController {

    public enum LatchLeftStatus {
        INITIALIZE,
        CLOSED,
        SECURED,
    }

    public latchLeftController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static LatchLeftStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double closed = 0.5;
    public static double secured = 0.95;

    public void update(robotMap r) {
        if (PS != CS || CS == INITIALIZE) {

            switch (CS) {

                case INITIALIZE: {
                    r.latchLeft.setPosition(closed);
                    break;
                }

                case CLOSED: {
                    r.latchLeft.setPosition(closed);
                    break;
                }

                case SECURED: {
                    r.latchLeft.setPosition(secured);
                    break;
                }
            }
        }

        PS = CS;

    }

}
