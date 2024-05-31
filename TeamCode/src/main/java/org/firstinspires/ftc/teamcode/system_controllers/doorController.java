package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.CLOSED;
import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.OPENED;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class doorController {

    public enum doorStatus
    {
        INITIALIZE,
        OPENED,
        CLOSED,
        CLOSED_COLLECT,
    }

    public doorController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static doorStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double opened = 0.84;
    public static double closed = 0.4;
    public static double closed_collect = 0.425;

    public void update(robotMap r)
    {
        if(CS != PS || CS == INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.door.setPosition(closed);
                    break;
                }

                case OPENED:
                {
                    r.door.setPosition(opened);
                    break;
                }

                case CLOSED:
                {
                    r.door.setPosition(closed);
                    break;
                }
                case  CLOSED_COLLECT:
                {
                    r.door.setPosition(closed_collect);
                    break;
                }
            }
        }

        PS = CS;
    }
}
