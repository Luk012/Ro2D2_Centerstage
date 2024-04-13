package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngle_i;
import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.CLOSED;
import static org.firstinspires.ftc.teamcode.system_controllers.doorController.doorStatus.CLOSED_COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.droneController.droneStatus.RELEASED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_BOTH;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_LEFT;
import static org.firstinspires.ftc.teamcode.system_controllers.latchDropController.latchDropStatus.DROP_ONE_RIGHT;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.UP;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_LIFT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_DOWN;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.HANG_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.OFF;
import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.ON;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLOSE_DOORv2;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DRIVE_POSE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;

import android.annotation.SuppressLint;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.dense.fixed.MatrixFeatures_DDF2;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

@Photon
@TeleOp(name="TeleOP", group="OpMode")
public class opMode extends LinearOpMode {

    /**
     * DRIVE
     */

    public static double  PrecisionDenominatorTranslational = 1, PrecisionDenominatorAngle = 1;

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double  SpeedLimit, boolean StrafesOn , double LeftTrigger, double RightTrigger, boolean pto_ON)
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

        if(pto_ON == true)
        {
            if(gamepad1.right_trigger > 0)
            {
                leftBack.setPower(-1);
                rightBack.setPower(-1);
            }
            else
            {
                leftBack.setPower(backLeftPower);
                rightBack.setPower(backRightPower);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);

        } else
        {
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }


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

        clawAngleController clawAngle = new clawAngleController();
        clawFlipController clawFlip = new clawFlipController();
        collectAngleController collectAngle = new collectAngleController();
        doorController door = new doorController();
        fourbarController fourbar = new fourbarController();
        latchLeftController latchLeft = new latchLeftController();
        latchRightController latchRight = new latchRightController();
        ptoController pto = new ptoController();
        droneController drone = new droneController();
        liftController lift = new liftController();
        extendoController extendo = new extendoController();
        transferController transfer = new transferController();
        outtakeController outtake = new outtakeController();
        latchDropController latchDrop = new latchDropController();

        double voltage;
        double loopTime = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        clawAngle.CS = clawAngleController.clawAngleStatus.INITIALIZE;
        clawFlip.CS = clawFlipController.clawFlipStatus.INITIALIZE;
        collectAngle.CS = collectAngleController.collectAngleStatus.INITIALIZE;
        door.CS = doorController.doorStatus.INITIALIZE;
        fourbar.CS = fourbarController.fourbarStatus.INITIALIZE;
        latchLeft.CS = latchLeftController.LatchLeftStatus.INITIALIZE;
        latchRight.CS = latchRightController.LatchRightStatus.INITIALIZE;
        pto.CS = ptoController.ptoStatus.INITIALIZE;
        lift.CS = DOWN;
        extendo.CS = extendoController.extendoStatus.INITIALIZE;
        drone.CS = droneController.droneStatus.INITIALIZE;


        clawAngle.update(r);
        clawFlip.update(r);
        clawAngle.update(r);
        door.update(r);
        fourbar.update(r);
        latchLeft.update(r);
        latchRight.update(r);
        pto.update(r);
        drone.update(r);
        lift.update(r, 0, voltage);
        extendo.update(r, 0, 1, voltage);
        transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo,lift);
        outtake.update(r, lift, fourbar, clawFlip, clawAngle, door, latchRight, latchLeft, transfer);
        latchDrop.update(r, latchRight, latchLeft, clawAngle);
        collectAngle.update(r);

        /**
         * OTHER INITS
         */

        boolean StrafesOn = true;
        boolean pto_ON = false;
        boolean ok = false;



        boolean drone_driver_1 = false;
        boolean drone_driver_2 = false;

        collectAngle_i = 0;

        double SpeedLimit = 1;

        extendo.extend_multiply_index = 0;

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


        ElapsedTime timer = new ElapsedTime();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

timer.reset();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            /**
             * INITS
             */

            MotorConfigurationType motorConfigurationType = r.leftBack.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.leftBack.setMotorType(motorConfigurationType);

            motorConfigurationType = r.rightBack.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.rightBack.setMotorType(motorConfigurationType);

            motorConfigurationType = r.rightFront.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.rightFront.setMotorType(motorConfigurationType);

            motorConfigurationType = r.leftFront.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.leftFront.setMotorType(motorConfigurationType);

            motorConfigurationType = r.extendoRight.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoRight.setMotorType(motorConfigurationType);

            motorConfigurationType = r.extendoLeft.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendoLeft.setMotorType(motorConfigurationType);

            motorConfigurationType = r.lift.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.lift.setMotorType(motorConfigurationType);

             position_lift = r.lift.getCurrentPosition();
            position_extendo = r.extendoLeft.getCurrentPosition();

            rightPixelState = r.pixelRight.getState();
            leftPixelState = r.pixelLeft.getState();



            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            robotCentricDrive(r.leftFront, r.leftBack, r.rightFront, r.rightBack, SpeedLimit , StrafesOn , 0,0, pto_ON);

            /**
             * COLLECT
             */

                collectPower = gamepad2.right_trigger - gamepad2.left_trigger;



            r.collect.setPower(collectPower);

            if(collectPower != 0)
            {
                collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
            } else
            {
                collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
            }

//            if((transfer.CS == TRANSFER_CLOSE_DOORv2 || transfer.CS == transferController.transferStatus.INITIALIZE))
//            {
//                if(collectPower == 0 || (!leftPixelState && !rightPixelState))
//                {
//                    door.CS = CLOSED;
//                } else
//                {
//                    door.CS = CLOSED_COLLECT;
//                }
//            }

            if(!previousGamepad2.left_stick_button && currentGamepad2.left_stick_button)
            {
               collectAngle.collectAngle_i = 0;
            }

            if(!previousGamepad2.right_stick_button && currentGamepad2.right_stick_button)
            {
                collectAngle.collectAngle_i = 4;
            }

            if(lift.CS == DOWN || lift.CS == INITIALIZE)
            {
            if(!previousGamepad2.dpad_right && currentGamepad2.dpad_right)
            {
                collectAngle.collectAngle_i = Math.min(4, collectAngle.collectAngle_i+1);
            }

            if(!previousGamepad2.dpad_left && currentGamepad2.dpad_left)
            {
                collectAngle.collectAngle_i = Math.max(0, collectAngle.collectAngle_i-1);
            }}

            /**
             * HEPTIC
             */

            if(!rightPixelState && !leftPixelState && collectPower > 0)
            {
                gamepad1.runRumbleEffect(effectCollect);
                gamepad2.runRumbleEffect(effectCollect);

                gamepad1.runLedEffect(rgbEffect);
                gamepad2.runLedEffect(rgbEffect);
            }




            /**
             * INTAKE
             */

            if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper && (transfer.CS == transferController.transferStatus.INITIALIZE || transfer.CS == TRANSFER_DONE || transfer.CS == TRANSFER_CLOSE_DOORv2))
            {
                if(extendo.CS != EXTENDED)
                {
                    extendo.CS = EXTENDED;
                } else
                {
                    extendo.CS = RETRACTED;
                }
            }

            if(!previousGamepad1.right_stick_button && currentGamepad1.right_stick_button)
            {
                if(door.CS != CLOSED)
                {
                    door.CS = CLOSED;
                }
            }

            if(!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
            {
                if(extendo.CS != DRIVE)
                {
                    extendo.CS = DRIVE;
                } else
                {
                    extendo.CS = RETRACTED;
                }
            }

            extend_power = gamepad1.right_trigger - gamepad1.left_trigger;

            if(extendo.CS == RETRACTED)
            {
                extendo.extend_multiply_index = 0;
            } else
            {
                extendo.extend_multiply_index += 10 * extend_power;
            }

            if(outtake.CS == outtakeController.outtakeStatus.INITIALIZE || outtake.CS == COLLECT_DONE)
            {  if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
                {
                    transfer.CS = TRANSFER_EXTENDO;
                }}


            /**
             * OUTTAKE
             */

            if(!previousGamepad2.cross && currentGamepad2.cross)
            {
                if(lift.CS != UP)
                {
                     PrecisionDenominatorAngle = 0.5;
                    outtake.CS = SCORE_FOURBAR;
                }
                else
                {
                    PrecisionDenominatorAngle = 1;
                 outtake.CS = COLLECT_FOURBAR;
                }
            }

            if(lift.CS == UP)
            {

                if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
            {
                lift.i_up = Math.min(20, lift.i_up+1);
            }

            if(!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
            {
                lift.i_up = Math.max(0, lift.i_up-1);
            }

                if(!previousGamepad2.dpad_right && currentGamepad2.dpad_right)
                {
                    lift.i_up = Math.min(20, lift.i_up+2);
                }

                if(!previousGamepad2.dpad_left && currentGamepad2.dpad_left)
                {
                    lift.i_up = Math.max(0, lift.i_up-2);
                }

            }

            if(lift.i_up > 11)
            {
                fourbar.score = 0.18;
            } else
            {
                fourbar.score = 0.18;
            }

            if(!previousGamepad2.right_bumper && currentGamepad2.right_bumper && lift.CS == UP)
            {
                clawAngle.clawAngle_i = Math.min(5, clawAngle.clawAngle_i+1);
            }

            if(!previousGamepad2.left_bumper && currentGamepad2.left_bumper && lift.CS == UP)
            {
                clawAngle.clawAngle_i = Math.max(0, clawAngle.clawAngle_i-1);
            }

            /**
             * LATCH CONTROL
             */

            if(previousGamepad2.triangle && currentGamepad2.triangle)
            {
                latchDrop.CS = DROP_BOTH;
                if(lift.CS == UP)
                {
                    gamepad1.runRumbleEffect(effectLatches);
                }
            }

            if(!previousGamepad2.circle && currentGamepad2.circle)
            {
                if(clawAngle.clawAngle_i <=2)
                {
                    if(latchLeft.CS != latchLeftController.LatchLeftStatus.CLOSED)
                    {
                        latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    } else
                    {
                        latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    }
                }
                else
                {
                    if(latchRight.CS != latchRightController.LatchRightStatus.CLOSED)
                    {
                        latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
                    } else
                    {
                        latchLeft.CS = latchLeftController.LatchLeftStatus.CLOSED;
                    }
                }
            }

            /**
             * END_GAME
             */

            if(gamepad1.left_trigger >0)
            {
                if(!previousGamepad1.dpad_up && currentGamepad1.dpad_up)
                {
                    lift.base += 10;
                }
                if(!previousGamepad1.dpad_down && currentGamepad1.dpad_down)
                {
                    lift.base -=10;
                }

                if(!previousGamepad1.dpad_right && currentGamepad1.dpad_right)
                {
                    extendo.retracted += 10;
                }
                if(!previousGamepad1.dpad_left && currentGamepad1.dpad_left)
                {
                 extendo.retracted -= 10;
                }
            } else
                {  if(!previousGamepad1.dpad_up && currentGamepad1.dpad_up)
            {
                if(pto.CS != ON)
                {
                    pto_ON = !pto_ON;
                  //  lift.CS = NOTHING;
                    pto.CS = ON;
                } else
                {
                    pto_ON = !pto_ON;
                    pto.CS = OFF;
                }
            } }

            if(!previousGamepad2.square && currentGamepad2.square)
            {
                if(outtake.CS != HANG_FOURBAR && outtake.CS != HANG_DONE)
                {
                    outtake.CS = HANG_FOURBAR;
                } else
                {
                    outtake.CS = HANG_DOWN;
                }
            }

            if(!previousGamepad1.touchpad && currentGamepad1.touchpad)
            {
                drone_driver_1 = !drone_driver_1;
            }

            if(!previousGamepad2.touchpad && currentGamepad2.touchpad)
            {
                drone_driver_2 = !drone_driver_2;
            }

            if(drone_driver_1 && drone_driver_2)
            {
                drone.CS = RELEASED;
            }




            clawAngle.update(r);
            clawFlip.update(r);
            clawAngle.update(r);
            door.update(r);
            fourbar.update(r);
            latchLeft.update(r);
            latchRight.update(r);
            pto.update(r);
            drone.update(r);
            collectAngle.update(r);
            lift.update(r, position_lift, voltage);
            extendo.update(r, position_extendo, 1, voltage);
            transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo, lift);
            outtake.update(r, lift, fourbar, clawFlip, clawAngle, door, latchRight, latchLeft, transfer);
            latchDrop.update(r, latchRight, latchLeft, clawAngle);

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            telemetry.addData("drone_driver_2", drone_driver_2);
//            telemetry.addData("drone_driver_1", drone_driver_1);
//            telemetry.addData("drone", drone.CS);
//            telemetry.addData("amps_extendoleft", r.extendoLeft.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("extendoright", r.extendoRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("liftamps", r.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("liftcp", r.lift.getCurrentPosition());
            telemetry.addData("lift", lift.activePID.targetValue );
            telemetry.addData("extendo", extendo.activePID.targetValue);
            telemetry.addData("status", outtake.CS);
            telemetry.addData("lift", lift.CS);
            telemetry.addData("transfer", transfer.CS);
            telemetry.addData("fourbarl", r.fourbarLeft.getPosition());
            telemetry.addData("fourbarr", r.fourbarRight.getPosition());
         //   telemetry.addData("fourbar_score", fourbar.score);


            loopTime = loop;


//            telemetry.addData("collectangle", collectAngle.CS);
//            telemetry.addData("collectanglei", collectAngle_i);
//            telemetry.addData("fourbarstatus", fourbar.CS);
//            telemetry.addData("liftpos", r.lift.getCurrentPosition());
//            telemetry.addData("outtake",outtake.CS);
//            telemetry.addData("transfer",transfer.CS);
//            telemetry.addData("amps extendo left", r.extendoLeft.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("amps extendo right", r.extendoRight.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("amps lift", r.lift.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());


            telemetry.update();
        }
    }
}