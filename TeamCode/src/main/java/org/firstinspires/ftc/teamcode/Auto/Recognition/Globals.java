package org.firstinspires.ftc.teamcode.Auto.Recognition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Globals {

    public enum Alliance
    {
        BLUE,
        RED,
    }

    public enum Side
    {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE,
    }

    public static Alliance alliance ;
    public static Side side;

    public static boolean isItNone ;
    public static boolean isItMiddle;

    public static Pose2d pose;
    public static AprilTagDetection aprilTagDetection;
    public static boolean canirelocalize;
    public static boolean waitformiddle;
    public static int desieredtag;
//    "bagpulaintactu"


    public static void returnAutoClawPoz(Alliance alliance, Side side)
    {
        if(alliance == Alliance.BLUE)
        {
            switch (side)
            {
                case LEFT:
                    clawAngleController.auto = clawAngleController.score[0];
                    break;
                default:
                    clawAngleController.auto = clawAngleController.score[1];
                    break;
            }
        } else
        {
            switch (side)
            {
                case LEFT:
                    clawAngleController.auto = clawAngleController.score[1];
                    break;
                default:
                    clawAngleController.auto = clawAngleController.score[0];
                    break;
            }
        }

        return;
    }

}
