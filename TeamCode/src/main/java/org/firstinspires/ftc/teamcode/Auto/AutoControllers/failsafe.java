package org.firstinspires.ftc.teamcode.Auto.AutoControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;

import java.io.BufferedReader;


public class failsafe {
    public enum failsafeStatus
    {
        NOTHING,
        FAIL_SAFE,
        FAIL_SAFE_HEADER,
        FAIL_SAFE_DONE,
        FAIL_SAFE_PURPLE,
        FAIL_SAFE_HEADER_PURPLE,
        FAIL_SAFE_DONE_PURPLE,
        FAIL_SAFE_NEAR,
        FAIL_SAFE_HEADER_NEAR,
        FAIL_SAFE_DONE_NEAR,

    }
    public static failsafeStatus CurrentStatus = failsafeStatus.NOTHING, PreviousStatus = failsafeStatus.NOTHING;

    ElapsedTime fail_safe_header = new ElapsedTime();


    public void update(robotMap r, liftController lift, fourbarController fourbar, clawAngleController clawAngle, clawFlipController clawFlip, collectAngleController collectAngle, doorController door, extendoController extendo, latchLeftController latchLeft, latchRightController latchRight)
    {

        switch (CurrentStatus)
        {

            case FAIL_SAFE:
            {
               // extendo.CS = extendoController.extendoStatus.FAIL_SAFE;
                r.collect.setPower(-0.7);
                fail_safe_header.reset();
                CurrentStatus = failsafeStatus.FAIL_SAFE_HEADER;
                break;
            }

            case FAIL_SAFE_HEADER:
            {

                if(fail_safe_header.seconds() > 0.05)
                {   r.collect.setPower(1);
                    collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i - 1);
                  //  extendo.CS = extendoController.extendoStatus.RETRY;
                    CurrentStatus = failsafeStatus.FAIL_SAFE_DONE;
                }
                break;
            }
            case FAIL_SAFE_PURPLE:
            {
                extendo.CS = extendoController.extendoStatus.FAIL_SAFE_PURPLE;
                r.collect.setPower(-0.7);
                fail_safe_header.reset();
                CurrentStatus = failsafeStatus.FAIL_SAFE_HEADER_PURPLE;
                break;
            }

            case FAIL_SAFE_HEADER_PURPLE:
            {

                if(fail_safe_header.seconds() > 0.35)
                {   r.collect.setPower(1);
                    collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i - 1);
                    extendo.CS = extendoController.extendoStatus.RERTRY_PURPLE;
                    CurrentStatus = failsafeStatus.FAIL_SAFE_DONE_PURPLE;
                }
                break;
            }

            case FAIL_SAFE_NEAR:
            {
                extendo.CS = extendoController.extendoStatus.FAIL_SAFE_NEAR;
                r.collect.setPower(-0.7);
                fail_safe_header.reset();
                CurrentStatus = failsafeStatus.FAIL_SAFE_HEADER_NEAR;
                break;
            }

            case FAIL_SAFE_HEADER_NEAR:
            {

                if(fail_safe_header.seconds() > 0.35)
                {   r.collect.setPower(1);
                    collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i - 1);
                    extendo.CS = extendoController.extendoStatus.RETRY_NEAR;
                    CurrentStatus = failsafeStatus.FAIL_SAFE_DONE_NEAR;
                }
                break;
            }




        }
        PreviousStatus = CurrentStatus;
    }
}

