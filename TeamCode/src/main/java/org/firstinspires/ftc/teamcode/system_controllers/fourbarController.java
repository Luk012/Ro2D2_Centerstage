package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.NEUTRAL;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.PRELOAD;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SCORE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class fourbarController {

    public enum fourbarStatus {
        SCORE,
        COLLECT,
        INITIALIZE,
        DRIVE,
        INTER,
        NEUTRAL,
        PRELOAD,
        SCORE_BAZI,
    }

    public fourbarController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static fourbarStatus CS = INITIALIZE, PS = INITIALIZE;

    public static  double init = 0.8;
    public static double score = 0.18;
    public static double scorebazi = 0.2;
    public static double collect = 0.93;
    public static double drive = 0.82;
    public static double preload = 0.075;
    public static double neutral = 0.5;

    public void update(robotMap r) {

        if(CS == SCORE)
        {
            r.fourbarLeft.setPosition(score);
            r.fourbarRight.setPosition(score);
        }

        if (PS != CS || CS == INITIALIZE || CS == COLLECT || CS == SCORE || CS == DRIVE || CS == NEUTRAL || CS == PRELOAD) {

            switch (CS) {

                case INITIALIZE: {
                    r.fourbarLeft.setPosition(init);
                    r.fourbarRight.setPosition(init);
                    break;
                }

                case COLLECT: {
                    r.fourbarLeft.setPosition(collect);
                    r.fourbarRight.setPosition(collect);
                    break;
                }

                case SCORE: {
                    r.fourbarLeft.setPosition(score);
                    r.fourbarRight.setPosition(score);
                    break;
                }

                case DRIVE:
                {
                    r.fourbarLeft.setPosition(drive);
                    r.fourbarRight.setPosition(drive);
                    break;
                }

                case NEUTRAL:
                {
                    r.fourbarLeft.setPosition(neutral);
                    r.fourbarRight.setPosition(neutral);
                    break;
                }

                case PRELOAD:
                {
                    r.fourbarLeft.setPosition(preload);
                    r.fourbarRight.setPosition(preload);
                    break;
                }

                case SCORE_BAZI:
                {
                    r.fourbarLeft.setPosition(scorebazi);
                    r.fourbarRight.setPosition(scorebazi);
                    break;
                }



            }
            PS = CS;
        }
    }

}
