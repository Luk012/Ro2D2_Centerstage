package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SCORE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class clawAngleController {

    public enum clawAngleStatus
    {
        INITIALIZE,
        COLLECT,
        SCORE,
    }

    public clawAngleController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static clawAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double collect = 0.5;
    public static double score[] = {0.123, 0.318, 0.5, 0.682, 0.877, 0.782, 0.782};


    public static int clawAngle_i = 1;

    public void update(robotMap r)
    {

        if(CS == SCORE)
        {
            r.clawAngle.setPosition(score[clawAngle_i]);
        }

        if(PS != CS || CS == INITIALIZE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.clawAngle.setPosition(collect);
                    break;
                }

                case COLLECT:
                {
                    r.clawAngle.setPosition(collect);
                    break;
                }

                case SCORE:
                {
                    r.clawAngle.setPosition(score[clawAngle_i]);
                    break;
                }


            }
        }

        PS = CS;
    }

}
