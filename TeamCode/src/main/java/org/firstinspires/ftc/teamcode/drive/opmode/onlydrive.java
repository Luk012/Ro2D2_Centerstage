package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngle_i;
import static org.firstinspires.ftc.teamcode.system_controllers.droneController.droneStatus.RELEASED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_BOTH;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.UP;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_LIFT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.OFF;
import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.ON;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.droneController;
import org.firstinspires.ftc.teamcode.system_controllers.latchDropController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
import org.firstinspires.ftc.teamcode.system_controllers.transferController;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.doorController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.ptoController;


@TeleOp(name="drivegen", group="OpMode")
public class onlydrive extends LinearOpMode {

    /**
     * DRIVE
     */

    public static double  PrecisionDenominatorTranslational = 1, PrecisionDenominatorAngle = 1;

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double  SpeedLimit, boolean StrafesOn , double LeftTrigger, double RightTrigger)
    {
        double y = -gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x;
        if (StrafesOn == false)
        {
            x=0;
        }
        double rx = gamepad1.left_stick_x*1 - LeftTrigger + RightTrigger;

        rx*=PrecisionDenominatorAngle;
        x/=PrecisionDenominatorTranslational;
        y/=PrecisionDenominatorTranslational;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,SpeedLimit);
        backLeftPower = Clip(backLeftPower,SpeedLimit);
        frontRightPower = Clip(frontRightPower,SpeedLimit);
        backRightPower = Clip(backRightPower,SpeedLimit);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    double Clip(double Speed, double lim)
    {
        return Math.max(Math.min(Speed,lim), -lim);
    }

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        robotMap r = new robotMap(hardwareMap);


        /**
         * SYSTEM CONTROLLERS
         */

//        clawAngleController clawAngle = new clawAngleController();
//        clawFlipController clawFlip = new clawFlipController();
//        collectAngleController collectAngle = new collectAngleController();
//        doorController door = new doorController();
//        fourbarController fourbar = new fourbarController();
//        latchLeftController latchLeft = new latchLeftController();
//        latchRightController latchRight = new latchRightController();
//        ptoController pto = new ptoController();
//        droneController drone = new droneController();
//        liftController lift = new liftController();
       extendoController extendo = new extendoController();
//        transferController transfer = new transferController();
//        outtakeController outtake = new outtakeController();
//        latchDropController latchDrop = new latchDropController();

        double voltage;
        double loopTime = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

//        clawAngle.CS = clawAngleController.clawAngleStatus.INITIALIZE;
//        clawFlip.CS = clawFlipController.clawFlipStatus.INITIALIZE;
//        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
//        door.CS = doorController.doorStatus.INITIALIZE;
//        fourbar.CS = fourbarController.fourbarStatus.INITIALIZE;
//        latchLeft.CS = latchLeftController.LatchLeftStatus.INITIALIZE;
//        latchRight.CS = latchRightController.LatchRightStatus.INITIALIZE;
//        pto.CS = ptoController.ptoStatus.INITIALIZE;
//        lift.CS = liftController.liftStatus.INITIALIZE;
     extendo.CS = extendoController.extendoStatus.INITIALIZE;
//        drone.CS = droneController.droneStatus.INITIALIZE;


//        clawAngle.update(r);
//        clawFlip.update(r);
//        clawAngle.update(r);
//        door.update(r);
//        fourbar.update(r);
//        latchLeft.update(r);
//        latchRight.update(r);
//        pto.update(r);
//        drone.update(r);
//        lift.update(r, 0, voltage);
      extendo.update(r, 0, 1, voltage);
//        transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo);
//        outtake.update(r, lift, fourbar, clawFlip, clawAngle);
//        latchDrop.update(r, latchRight, latchLeft, clawAngle);

        /**
         * OTHER INITS
         */

        boolean StrafesOn = true;

        boolean drone_driver_1 = false;
        boolean drone_driver_2 = false;

        double SpeedLimit = 1;

//        extendo.extend_multiply_index = 0;

        int position_lift;
        int position_extendo;

        boolean rightPixelState;
        boolean leftPixelState;

        Gamepad.RumbleEffect effectCollect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 50)
                .addStep(1.0, 1.0, 100)
                .build();

        Gamepad.RumbleEffect effectLatches = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 250)
                .addStep(0.0, 0.0, 50)
                .addStep(1.0, 0.0, 100)
                .build();

        Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 50) // Show red for 250ms
                .addStep(0, 1, 0, 50) // Show green for 250ms
                .addStep(0, 0, 1, 50) // Show blue for 250ms
                .build();

        double collectPower = 0;
        double extend_power = 0;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            /**
             * INITS
             */

//            position_lift = r.lift.getCurrentPosition();
          position_extendo = r.extendoLeft.getCurrentPosition();
//
//            rightPixelState = r.pixelRight.getState();
//            leftPixelState = r.pixelLeft.getState();



            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


           // robotCentricDrive(r.leftFront, r.leftBack, r.rightFront, r.rightBack, SpeedLimit , StrafesOn , 0,0);

            double power = gamepad1.right_trigger - gamepad1.left_trigger;

            r.leftBack.setPower(power);
            r.rightBack.setPower(power);

            if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
            {
                if(extendo.CS != EXTENDED)
                {
                    extendo.CS = EXTENDED;
                } else
                {
                    extendo.CS = RETRACTED;
                }
            }
//
//            /**
//             * COLLECT
//             */
//
//            if(gamepad2.left_trigger > 0)
//            {
//                collectPower = gamepad2.right_trigger * -1;
//            } else {
//                collectPower = gamepad2.right_trigger;
//            }
//
//            r.collect.setPower(collectPower);
//
//            if(collectPower != 0)
//            {
//                collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//            } else
//            {
//                collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//            }
//
//            if(!previousGamepad2.left_stick_button && currentGamepad2.left_stick_button)
//            {
//                collectAngle_i = 0;
//            }
//
//            if(!previousGamepad2.right_stick_button && currentGamepad2.right_stick_button)
//            {
//                collectAngle_i = 4;
//            }
//
//            if(!previousGamepad2.dpad_right && currentGamepad2.dpad_right)
//            {
//                collectAngle.collectAngle_i = Math.min(4, collectAngle.collectAngle_i+1);
//            }
//
//            if(!previousGamepad2.dpad_left && currentGamepad2.dpad_left)
//            {
//                collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
//            }
//
//            /**
//             * HEPTIC
//             */
//
//            if(rightPixelState && leftPixelState && collectPower > 0)
//            {
//                gamepad1.runRumbleEffect(effectCollect);
//                gamepad2.runRumbleEffect(effectCollect);
//
//                gamepad1.runLedEffect(rgbEffect);
//                gamepad2.runLedEffect(rgbEffect);
//            }
//
//            /**
//             * INTAKE
//             */
//
//            if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
//            {
//                if(extendo.CS != EXTENDED)
//                {
//                    extendo.CS = EXTENDED;
//                } else
//                {
//                    extendo.CS = RETRACTED;
//                }
//            }
//
//            extend_power = gamepad1.right_trigger - gamepad1.left_trigger;
//
//            if(extendo.CS == RETRACTED)
//            {
//                extendo.extend_multiply_index = 0;
//            } else
//            {
//                extendo.extend_multiply_index += 10 * extend_power;
//            }
//
//            if(gamepad2.left_trigger > 0)
//            {
//                if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
//                {
//                    transfer.CS = TRANSFER_EXTENDO;
//                }
//            }
//
//            /**
//             * OUTTAKE
//             */
//
//            if(!previousGamepad2.cross && currentGamepad2.cross)
//            {
//                if(lift.CS != UP)
//                {
//                    outtake.CS = SCORE_FOURBAR;
//                }
//                else
//                {
//                    outtake.CS = COLLECT_LIFT;
//                }
//            }
//
//            if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
//            {
//                lift.i_up = Math.min(20, lift.i_up+1);
//            }
//
//            if(!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
//            {
//                lift.i_up = Math.max(0, lift.i_up-1);
//            }
//
//            if(!previousGamepad2.right_bumper && currentGamepad2.right_bumper && lift.CS == UP)
//            {
//                clawAngle.clawAngle_i = Math.min(4, clawAngle.clawAngle_i+1);
//            }
//
//            if(!previousGamepad2.left_bumper && currentGamepad2.left_bumper && lift.CS == UP)
//            {
//                clawAngle.clawAngle_i = Math.max(0, clawAngle.clawAngle_i-1);
//            }
//
//            /**
//             * LATCH CONTROL
//             */
//
//            if(previousGamepad2.triangle && currentGamepad2.triangle)
//            {
//                latchDrop.CS = DROP_BOTH;
//                if(lift.CS == UP)
//                {
//                    gamepad1.runRumbleEffect(effectLatches);
//                }
//            }
//
//            if(!previousGamepad2.circle && currentGamepad2.circle)
//            {
//                latchDrop.CS = DROP_ONE;
//            }
//
//            /**
//             * END_GAME
//             */
//
//            if(!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
//            {
//                if(pto.CS != ON)
//                {
//                    pto.CS = ON;
//                } else
//                {
//                    pto.CS = OFF;
//                }
//            }
//
//            if(!previousGamepad1.touchpad && currentGamepad1.touchpad)
//            {
//                drone_driver_1 = !drone_driver_1;
//            }
//
//            if(!previousGamepad2.touchpad && currentGamepad2.touchpad)
//            {
//                drone_driver_2 = !drone_driver_2;
//            }
//
//            if(drone_driver_1 && drone_driver_2)
//            {
//                drone.CS = RELEASED;
//            }
//
//
//
//
//            clawAngle.update(r);
//            clawFlip.update(r);
//            clawAngle.update(r);
//            door.update(r);
//            fourbar.update(r);
//            latchLeft.update(r);
//            latchRight.update(r);
//            pto.update(r);
//            drone.update(r);
//            lift.update(r, position_lift, voltage);
           extendo.update(r, position_extendo, 1, voltage);
//            transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo);
//            outtake.update(r, lift, fourbar, clawFlip, clawAngle);
//            latchDrop.update(r, latchRight, latchLeft, clawAngle);

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));

            loopTime = loop;


//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());


            telemetry.update();
        }
    }
}
