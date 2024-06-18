package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Photon
@Config
@Autonomous(group = "Testers")
public class PidControllerTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double Kp = 0.00325;
    public static double Ki = 0.0022;
    public static double Kd = 0;
    public static double maxSpeed = 1;
    public static double RetractedPosition = 0 , ExtendedPosition = 300;
    public static double vMax = 0, AccMax = 0, JerkMax =0 , EndPos = 700;
    int TargetLift = 0;
    ElapsedTime timerPID = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {
        List <LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double loopTime = 0;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ElapsedTime changePositions = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotMap robot = new robotMap(hardwareMap);
        SimplePIDController hello = new SimplePIDController(Kp,Ki,Kd);
        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime now = new ElapsedTime();
        now.reset();
        telemetry.update();
        hello.targetValue = RetractedPosition;
        while (!isStopRequested() && opModeIsActive())
        {
            int ColectarePosition = robot.extendoLeft.getCurrentPosition();
            double powerColectare = hello.update(ColectarePosition);
            powerColectare = Math.max(-1,Math.min(powerColectare,1));
            robot.extendoLeft.setPower(powerColectare);
            robot.extendoRight.setPower(powerColectare);
           // robot.extendoRight.setPower(powerColectare);
            if (changePositions.seconds()>4)
            {
                if (hello.targetValue == RetractedPosition )
                {
                    hello.targetValue = ExtendedPosition;
                }
                else
                {
                    hello.targetValue = RetractedPosition;
                }
                changePositions.reset();
            }
            telemetry.addData("ColectareEncoder", ColectarePosition);
            telemetry.addData("powerColectare", powerColectare);
            telemetry.addData("TargetLift",hello.targetValue);
            telemetry.addData("Error", hello.measuredError(ColectarePosition));

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));

            loopTime = loop;


//            telemetry.addData("distance 1", robot.pixelLeft.getState());
//            telemetry.addData("distance2", robot.pixelRight.getState());
//            telemetry.addData("distance3", robot.back.getDistance(DistanceUnit.CM));
//            telemetry.addData("distance4", robot.extendoDistance.getDistance(DistanceUnit.CM));
//            telemetry.addData("encoder1", robot.leftBack.getCurrentPosition());
//            telemetry.addData("encoder2", robot.leftFront.getCurrentPosition());
//            telemetry.addData("encoder3", robot.rightBack.getCurrentPosition());
//            telemetry.addData("encoder4", robot.rightFront.getCurrentPosition());
            if (Kp!=hello.p || Kd!=hello.d || Ki!=hello.i || maxSpeed !=hello.maxOutput )
            {
                hello.p = Kp;
                hello.d = Kd;
                hello.i = Ki;
                hello.maxOutput = maxSpeed;
            }

            telemetry.update();
        }
    }
}