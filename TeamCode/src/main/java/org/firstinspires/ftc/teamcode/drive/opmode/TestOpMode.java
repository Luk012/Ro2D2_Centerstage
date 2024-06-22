//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import android.annotation.SuppressLint;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
//import org.firstinspires.ftc.teamcode.globals.robotMap;
//import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
//import org.firstinspires.ftc.teamcode.system_controllers.clawFlipController;
//import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
//import org.firstinspires.ftc.teamcode.system_controllers.doorController;
//import org.firstinspires.ftc.teamcode.system_controllers.droneController;
//import org.firstinspires.ftc.teamcode.system_controllers.droneLatchController;
//import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
//import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
//import org.firstinspires.ftc.teamcode.system_controllers.latchDropController;
//import org.firstinspires.ftc.teamcode.system_controllers.latchLeftController;
//import org.firstinspires.ftc.teamcode.system_controllers.latchRightController;
//import org.firstinspires.ftc.teamcode.system_controllers.liftController;
//import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
//import org.firstinspires.ftc.teamcode.system_controllers.ptoController;
//import org.firstinspires.ftc.teamcode.system_controllers.transferController;
//
//import java.util.concurrent.TimeoutException;
//
//
//@TeleOp(name="muie bravebotz", group="OpMode")
//public class TestOpMode extends LinearOpMode {
//
//
//
//    @SuppressLint("SuspiciousIndentation")
//    @Override
//    public void runOpMode() {
//        robotMap r = new robotMap(hardwareMap);
//
//    //  clawAngleController clawAngle = new clawAngleController();
//    //clawFlipController clawFlip = new clawFlipController();
//       //collectAngleController collectAngle = new collectAngleController();
// doorController door = new doorController();
//      // fourbarController fourbar = new fourbarController();
//    // latchLeftController latchLeft = new latchLeftController();
// //latchRightController latchRight = new latchRightController();
// //    ptoController pto = new ptoController();
//     // droneController drone = new droneController();
////        liftController lift = new liftController();
//   //  extendoController extendo = new extendoController();
//     //  transferController transfer = new transferController();
////        outtakeController outtake = new outtakeController();
////        latchDropController latchDrop = new latchDropController();
//      //  droneLatchController droneLatch = new droneLatchController();
//
//
////lift.CS = lift_Controller.liftStatus.DOWN;
//
//        double voltage;
//        double loopTime = 0;
//        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//        voltage = batteryVoltageSensor.getVoltage();
//
//
////storageAngle.CS = storageAngle_Controller.storageAngleStatus.ROTATION;
//
//
//    //   lift.update(r, 0, voltage);
////door.update(r);
//
//
//
//        boolean StrafesOn = true;
//        boolean stack = false;
//
//        //lift.upCnt = 0;
//       // storageAngle.rotation_i = 0;
//
//
//
//        double SpeedLimit = 1;
//
//        Gamepad currentGamepad1 = new Gamepad();
//        Gamepad currentGamepad2 = new Gamepad();
//
//        Gamepad previousGamepad1 = new Gamepad();
//        Gamepad previousGamepad2 = new Gamepad();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//
//        double collect_power = 0;
//
//        waitForStart();
//
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//          //  int position = r.extendoRight.getCurrentPosition();
//
//
//
//
//            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);
//
//            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);
////
//            if(!previousGamepad1.circle && currentGamepad1.circle)
//            {
//               if(door.CS != doorController.doorStatus.OPENED)
//               {
//                   door.CS = doorController.doorStatus.OPENED;
//               } else
//               {
//                   door.CS = doorController.doorStatus.CLOSED;
//               }
//            }
////            if(!previousGamepad1.triangle && currentGamepad1.triangle)
////            {
////              if(latchRight.CS != latchRightController.LatchRightStatus.SECURED)
////              {
////                  latchRight.CS = latchRightController.LatchRightStatus.SECURED;
////              } else
////              {
////                  latchRight.CS = latchRightController.LatchRightStatus.CLOSED;
////              }
////            }
////
////
//              double power = gamepad1.right_trigger - gamepad1.left_trigger;
//
//              r.collect.setPower(power);
//
//
//
//           // drone.update(r);
//
//// clawAngle.update(r);
//          //  clawFlip.update(r);
////            clawAngle.update(r);
//          //door.update(r);
//    //   collectAngle.update(r);
////        droneLatch.update(r);
////        latchLeft.update(r);
////            latchRight.update(r);
//        //  pto.update(r);
//  // drone.update(r);
////            lift.update(r, 0, voltage);
//          //  extendo.update(r, position, 1, voltage);
//    //  transfer.update(r, door, fourbar, clawAngle, clawFlip, latchLeft, latchRight, extendo);
////            outtake.update(r, lift, fourbar, clawFlip, clawAngle);
////            latchDrop.update(r, latchRight, latchLeft, clawAngle);
//
//            double loop = System.nanoTime();
//
//            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            telemetry.addData("latchLeft",  door.CS );
//
//
//          //  telemetry.addData("position", position);
//            loopTime = loop;
////            telemetry.addData("x", poseEstimate.getX());
////            telemetry.addData("y", poseEstimate.getY());
////            telemetry.addData("heading", poseEstimate.getHeading());
//
//
//
////            telemetry.addData("leftPixel", r.pixelLeft.getState());
////            telemetry.addData("rightPixel", r.pixelRight.getState());
////            telemetry.addData("extendo", r.extendoDistance.getDistance(DistanceUnit.MM));
//          // telemetry.addData("fourbar", fourbar.CS);
//         //  telemetry.addData("tick-uri",r.extendoLeft.getCurrentPosition());
//           // telemetry.addData("distance", r.extendoDistance.getDistance(DistanceUnit.MM));
//          //  telemetry.addData("encoder", r.lift.getCurrentPosition());
//
//
//            telemetry.update();
//        }
//    }
//}
//
