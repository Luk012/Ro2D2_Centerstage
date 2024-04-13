package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.droneController.droneStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class droneController {

    public enum droneStatus
    {
        INITIALIZE,
        SECURED,
        RELEASED,
    }

    public droneController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static droneStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double secured = 0.5;
    public static double released = 1;

    public void update(robotMap r)
    {
        if(CS != PS || CS == INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.drone.setPosition(secured);
                    break;
                }

                case SECURED:
                {
                    r.drone.setPosition(secured);
                    break;
                }

                case RELEASED:
                {
                    r.drone.setPosition(released);
                    break;
                }
            }
        }

        PS = CS;
    }

}
