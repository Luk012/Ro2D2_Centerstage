package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Auto.BlueFar;
import org.firstinspires.ftc.teamcode.Auto.RedNearLinear;
import org.firstinspires.ftc.teamcode.drive.opmode.BackAndForth;
import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class liftController {

    public enum liftStatus
    {
        INITIALIZE,
        DOWN,
        UP,
        CYCLE,
        HANG,
        YELLOW_NEAR,

        PRELOAD_YELLOW,
        NOTHING,
        TRANSFER,
        ALILBIT_UP,
    }

    public double CurrentSpeed = 0;

    /**
     * UP
     */


    public static double PAUTO = 0.0045;
    public static double IAUTO = 0;
    public static double DAUTO = 0.001;
    //
    public static double PDRIVE = 0.0075;
    public static double IDRIVE = 0.0012;
    public static double DDRIVE = 0.001;

    public static double PDOWN = 0.0095;
    public static double IDOWN = 0;
    public static double DDOWN = 0;

    int target;

    public double pid = 0;

    public double Kg = 0;
    public double maxSpeedUp = 1;

    public static liftStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController LiftPID_AUTO = null;
    SimplePIDController LiftPIDDOWN = null;
    SimplePIDController LiftPID_DRIVE = null;
    public SimplePIDController activePID;



    public static double base = -40;

    public static double down = 108;

    public static int i_up = 0;
    public static double i_multiplication = 54;

    public static double hang = 690;
    public static double cycle = 400;

    public int CurrentPosition = 0;
    public int error = 2;
    public static double transfer = -100;

    public liftController()
    {
        LiftPID_AUTO = new SimplePIDController(PAUTO,IAUTO,DAUTO);
        LiftPID_AUTO.targetValue = base;
        LiftPID_AUTO.maxOutput = maxSpeedUp;

        LiftPIDDOWN = new SimplePIDController(PDOWN,IDOWN,DDOWN);
        LiftPIDDOWN.targetValue = base;
        LiftPIDDOWN.maxOutput = maxSpeedUp;

        LiftPID_DRIVE = new SimplePIDController(PDRIVE,IDRIVE,DDRIVE);
        LiftPID_DRIVE.targetValue = base;
        LiftPID_DRIVE.maxOutput = maxSpeedUp;
    }


    public void disableMotor(robotMap r) {
        r.lift.setMotorDisable();
    }

    public void enableMotor(robotMap r)
    {
        r.lift.setMotorEnable();
    }

    public void update(robotMap r, int position, double voltage)
    {
        switch (CS) {
            case UP: // Define your conditions
                activePID = LiftPID_DRIVE;
                break;
            case DOWN:
                activePID = LiftPIDDOWN;
                break;
            case CYCLE:
                activePID = LiftPID_AUTO;
                break;
            case PRELOAD_YELLOW:
                activePID = LiftPID_AUTO;
                break;
            case YELLOW_NEAR:
                activePID = LiftPID_AUTO;
                break;
            case ALILBIT_UP:
                activePID = LiftPID_AUTO;
                break;
            case HANG:
                activePID = LiftPID_DRIVE;
                break;
            default:
                activePID = LiftPIDDOWN; // Default to the first one or any you prefer
                break;
        }

        CurrentPosition = position;
        double powerLift = activePID.update(position) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift* 14 / voltage,1));
        CurrentSpeed=powerLift;


      //  r.lift.setPower(powerLift);

        if (activePID.targetValue <= 0 && r.lift.getCurrentPosition() <=  1 && CS == DOWN) {
           r.lift.setPower(0);
        } else
        if(activePID.targetValue > 0 || r.lift.getCurrentPosition() > 1 || CS == TRANSFER)
        {
            r.lift.setPower(powerLift);
        }

        if(CS == liftStatus.UP) activePID.targetValue = down + i_up * i_multiplication;

        if(CS == INITIALIZE || CS == DOWN)
            activePID.targetValue = base;

        if(CS != PS || CS == INITIALIZE )
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                 //   target = -5;
                    activePID.targetValue = base;
                    break;
                }

                case UP:
                {
                 //   target = 10;
                    activePID.targetValue = down + i_up * i_multiplication;
                    break;
                }

                case DOWN:
                {
                   // target = -10;
                    activePID.targetValue = base;
                    break;
                }

                case PRELOAD_YELLOW:
                {
                   // target = 10;
                    activePID.targetValue = 300;
                    break;
                }

                case  YELLOW_NEAR:
                {
                    if(BlueFar.caz == 0)
                    {  activePID.targetValue = 190;}
                    else
                    {
                        activePID.targetValue = 190;
                    }
                    break;
                }

                case CYCLE:
                {
                    activePID.targetValue = cycle;
                    break;
                }

                case HANG:
                {//target = 0;
                    activePID.targetValue = hang;
                    break;
                }

                case TRANSFER:
                {
                   // target = -30;
                    activePID.targetValue = transfer;
                    break;
                }

                case ALILBIT_UP:
                {
                    activePID.targetValue = cycle + 100;
                    break;
                }
            }
        }

        PS = CS;
    }

}