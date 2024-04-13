package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeoutException;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Testers")
public class testliftgen extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double Kp = 0;// 0.006
    public static double Ki = 0; // 0.0045
    public static double Kd = 0;
    public static double Kg = 0;
    public static double maxSpeed = 1;
    public static double plm=0;
    public static double RetractedPosition = 0 , ExtendedPosition = 600;
    int TargetLift = 0;
    ElapsedTime timerPID = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {
        List <LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double loopTime = 0;

        OpenCvWebcam test = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        test.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                //test.setPipeline(redLeft);

                test.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

                //while (!redLeft.hasProcessedFrame || !yellowPixel.hasProcessedFrame) sleep(50);

                //itemStatus = redLeft.getWhichSide();

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        ElapsedTime changePositions = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotMap robot = new robotMap(hardwareMap);
        fourbarController fourbar = new fourbarController();
        SimplePIDController hello = new SimplePIDController(Kp,Ki,Kd);
        waitForStart();

        if (isStopRequested()) return;


        telemetry.update();
        hello.targetValue = RetractedPosition;

        while (!isStopRequested() && opModeIsActive())
        {
            // if(plm==0)
            // SigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;

            fourbar.CS = fourbarController.fourbarStatus.NEUTRAL;

            int DreaptaLiftPosition = robot.lift.getCurrentPosition();
            double powerDreaptaLift = hello.update(DreaptaLiftPosition) + Kg;
            powerDreaptaLift = Math.max(-1,Math.min(powerDreaptaLift,1));
            robot.lift.setPower(powerDreaptaLift);
            if (changePositions.seconds()>3)
            {
                if (hello.targetValue == RetractedPosition )
                {
                    hello.targetValue = ExtendedPosition;
                    // SigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                else
                {
                    hello.targetValue = RetractedPosition;
                }
                changePositions.reset();
            }

            fourbar.update(robot);


            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));

            loopTime = loop;

            telemetry.addData("ColectareEncoder", DreaptaLiftPosition);
            telemetry.addData("powerColectare", powerDreaptaLift);
            telemetry.addData("TargetLift",hello.targetValue);
            telemetry.addData("Error", hello.measuredError(DreaptaLiftPosition));
            telemetry.addData("distance 1", robot.pixelLeft.getState());
            telemetry.addData("distance2", robot.pixelRight.getState());
//            telemetry.addData("distance3", robot.back.getDistance(DistanceUnit.CM));
//            telemetry.addData("distance4", robot.extendoDistance.getDistance(DistanceUnit.CM));
//            telemetry.addData("encoder1", robot.leftBack.getCurrentPosition());
            telemetry.addData("encoder2", robot.leftFront.getCurrentPosition());
            telemetry.addData("encoder3", robot.rightBack.getCurrentPosition());
            telemetry.addData("encoder4", robot.rightFront.getCurrentPosition());
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
