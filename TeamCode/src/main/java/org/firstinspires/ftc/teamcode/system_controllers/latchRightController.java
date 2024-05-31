package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.latchRightController.LatchRightStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class latchRightController {

    public enum LatchRightStatus
    {
        INITIALIZE,
        CLOSED,
        SECURED,
    }

    public latchRightController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static LatchRightStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double closed = 0.95;
    public static double secured = 0.56;

    public void update(robotMap r)
    {
        if(CS != PS || CS == INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.latchRight.setPosition(closed);
                    break;
                }

                case SECURED:
                {
                    r.latchRight.setPosition(secured);
                    break;
                }

                case CLOSED:
                {
                    r.latchRight.setPosition(closed);
                    break;
                }
            }
        }

        PS = CS;
    }

}
