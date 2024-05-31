package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.droneController.droneStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class droneLatchController {

    public enum droneLatchStatus
    {
        INITIALIZE,
        SECURED,
        RELEASED,
    }

    public droneLatchController()
    {
        CS = droneLatchStatus.INITIALIZE;
        PS = droneLatchStatus.INITIALIZE;
    }

    public static droneLatchStatus CS = droneLatchStatus.INITIALIZE, PS = droneLatchStatus.INITIALIZE;

    public static double secured = 0.247;
    public static double released = 1;

    public void update(robotMap r)
    {
        if(CS != PS || CS == droneLatchStatus.INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.droneLatch.setPosition(secured);
                    break;
                }

                case SECURED:
                {
                    r.droneLatch.setPosition(secured);
                    break;
                }

                case RELEASED:
                {
                    r.droneLatch.setPosition(released);
                    break;
                }
            }
        }

        PS = CS;
    }

}
