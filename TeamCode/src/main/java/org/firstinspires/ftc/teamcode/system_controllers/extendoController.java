package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.CYCLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.FAIL_SAFE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.PURPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Auto.BlueFar;
import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class extendoController {

    public enum extendoStatus
    {
        INITIALIZE,
        RETRACTED,
        EXTENDED,
        PURPLE,
        CYCLE,
        DRIVE,
        FAIL_SAFE,
        TRANSFER,
        RETRY,
        RERTRY_PURPLE,
        FAIL_SAFE_PURPLE,
    }

    // PID constants for extension
    public static double Kp_extend = 0.006;
    public static double Ki_extend = 0.001;
    public static double Kd_extend = 0.002;

    // PID constants for retraction
    public static double Kp_retract = 0.015; // Example values, adjust based on your needs
    public static double Ki_retract = 0;
    public static double Kd_retract = 0;

    public static double Kp_drive = 0.0025; // Example values, adjust as needed
    public static double Ki_drive = 0.001;
    public static double Kd_drive = 0.002;



    SimplePIDController extendoPIDExtend;
    SimplePIDController extendoPIDRetract;
    SimplePIDController extendoPIDDrive;

//
//    public static double Kp = 0.0031;
//    public static double Ki = 0;
//    public static double Kd = 0;



    public static double maxSpeed = 1;

    public extendoStatus CS = INITIALIZE, PS = INITIALIZE;

    //   SimplePIDController extendoPID = null;

    public static double CurrentPosition = 0;
    public static double retracted = -5;
    public static double extended = 900;
    public static double drive = 600;
    public static double failsafe = 790;
    public static double purple[] = {500, 250, 0};
    public static double cycle = 945;
    public static double cycle_far = 890;
    public static double x = 10;
    public static int caz = 0;
    public static double transfer = -40;
    public static double retry = 980;
    public static double retry_purple[] = {580, 340, 0};
    public static double fail_purple[] = {400, 150, 0};


    public static double extend_multiply_index = 0;

    public extendoController()
    {
        extendoPIDExtend = new SimplePIDController(Kp_extend, Ki_extend, Kd_extend);
        extendoPIDRetract = new SimplePIDController(Kp_retract, Ki_retract, Kd_retract);
        extendoPIDDrive = new SimplePIDController(Kp_drive, Ki_drive, Kd_drive);

        // Initially, we can set to retract as a default or based on your initial state
        extendoPIDExtend.targetValue = retracted; // Assuming you start with retraction
        extendoPIDExtend.maxOutput = maxSpeed;

        extendoPIDRetract.targetValue = retracted;
        extendoPIDRetract.maxOutput = maxSpeed;

        extendoPIDDrive.targetValue = retracted;
        extendoPIDDrive.maxOutput = maxSpeed;
    }

    public void update(robotMap r, int position, double powerCap, double voltage)
    {

        SimplePIDController activePID;
        switch (CS) {
            case INITIALIZE:
                activePID = extendoPIDRetract;
                break;
            case EXTENDED: // Define your conditions
                activePID = extendoPIDDrive;
                break;
            case RETRACTED:
                activePID = extendoPIDRetract;
                break;
            case PURPLE:
                activePID = extendoPIDDrive;
                break;
            case CYCLE:
                activePID = extendoPIDDrive;
                break;
            case DRIVE:
                activePID = extendoPIDDrive;
                break;
            case FAIL_SAFE:
                activePID = extendoPIDExtend;
                break;
            case RETRY:
                activePID = extendoPIDExtend;
                break;
            case FAIL_SAFE_PURPLE:
                activePID = extendoPIDExtend;
                break;
            case RERTRY_PURPLE:
                activePID = extendoPIDExtend;
                break;
            default:
                activePID = extendoPIDRetract; // Default to the first one or any you prefer
                break;
        }

        // Use the active PID controller for calculations
        double powerColectare = activePID.update(position);
        powerColectare = Math.max(-powerCap, Math.min(powerColectare * 14 / voltage, powerCap));

        r.extendoLeft.setPower(powerColectare);
        r.extendoRight.setPower(powerColectare);

        if(CS == EXTENDED)
        {
            activePID.targetValue = extended + extend_multiply_index;
        }

        if(CS == FAIL_SAFE)
        {
            activePID.targetValue = failsafe + x;
        }

        if(CS == RETRACTED || CS == INITIALIZE)
        {
            activePID.targetValue = retracted;
        }

//          if(CS == SENSOR && CurrentPosition >= auto[ciclu] - 10)
//          {
//              extendoPID.targetValue = difference;
//              extendoPID.maxOutput = 0.6;
//          }

        if(CS != PS || CS == INITIALIZE || CS == EXTENDED || CS == RETRACTED || CS == PURPLE || CS == FAIL_SAFE || CS == DRIVE || CS == CYCLE)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED:
                {
                    activePID.targetValue = extended + extend_multiply_index;
                    activePID.maxOutput = 1;
                    break;
                }

                case RETRY:
                {
                    activePID.targetValue = retry;
                    activePID.maxOutput = 1;
                    break;
                }

                case RETRACTED:
                {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case PURPLE:
                {
                    activePID.targetValue = purple[caz];
                    activePID.maxOutput = 0.9;
                    //CS = SENSOR;
                    break;
                }

                case CYCLE:
                {
                    activePID.targetValue = cycle_far;
                    activePID.maxOutput = 1;
                    break;
                }

                case DRIVE:
                {
                    activePID.targetValue = drive;
                    activePID.maxOutput = 1;
                    break;
                }

                case FAIL_SAFE:
                {
                    activePID.targetValue = failsafe + x;
                    activePID.maxOutput = 1;
                    break;
                }

                case TRANSFER:
                {
                    activePID.targetValue = transfer;
                    activePID.maxOutput =1;
                    break;
                }

                case RERTRY_PURPLE:
                {
                    activePID.targetValue = retry_purple[BlueFar.caz];
                    activePID.maxOutput =1;
                    break;
                }

                case FAIL_SAFE_PURPLE:
                {
                    activePID.targetValue = fail_purple[BlueFar.caz];
                    activePID.maxOutput =1;
                    break;
                }
            }

            PS = CS;
        }

    }

}