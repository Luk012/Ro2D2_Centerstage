package org.firstinspires.ftc.teamcode.system_controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class droneAssemblyController {

    public enum droneAssemblyStatus
    {
        INITIALIZE,
        DRONE_LATCH,
        DRONE_LAUNCH,
        DRONE_TIME_RESET,
        DRONE_DONE,

    }

    public droneAssemblyController()
    {
        CS = droneAssemblyStatus.INITIALIZE;
        PS = droneAssemblyStatus.INITIALIZE;
    }

    public static droneAssemblyStatus CS = droneAssemblyStatus.INITIALIZE, PS = droneAssemblyStatus.INITIALIZE;

  public ElapsedTime drone_launch = new ElapsedTime();

    public void update(droneController drone, droneLatchController droneLatch)
    {
        if(CS != PS || CS == droneAssemblyStatus.INITIALIZE || CS == droneAssemblyStatus.DRONE_DONE || CS == droneAssemblyStatus.DRONE_LATCH || CS == droneAssemblyStatus.DRONE_LAUNCH || CS == droneAssemblyStatus.DRONE_TIME_RESET)
        {
            switch (CS)
            {
                case DRONE_LATCH:
                {
                    droneLatch.CS = droneLatchController.droneLatchStatus.RELEASED;
                    CS = droneAssemblyStatus.DRONE_TIME_RESET;
                    break;
                }

                case DRONE_TIME_RESET:
                {
                    drone_launch.reset();
                    CS = droneAssemblyStatus.DRONE_LAUNCH;
                    break;
                }

                case DRONE_LAUNCH:
                {
                    if(drone_launch.seconds() > 0.15)
                    {
                        drone.CS = droneController.droneStatus.RELEASED;

                    }
                    if(drone_launch.seconds() > 0.3)
                    {
                        CS = droneAssemblyStatus.DRONE_DONE;
                    }
                    break;
                }

            }
        }

        PS = CS;
    }

}
