package org.firstinspires.ftc.teamcode.Auto.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;


@Disabled
@Autonomous(name = "YellowPixelTest")
public class YellowPixelTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        YellowPixelMaster yellowTest = new YellowPixelMaster(this);
     //   ConceptAprilTag test = new ConceptAprilTag();

        yellowTest.observeStick();

        telemetry.addData("Item: ", yellowTest.yellowPixel.getWhichSide());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
          //  test.runOpMode();
            telemetry.addData("Item: ", yellowTest.yellowPixel.getWhichSide());
            telemetry.addData("imx", yellowTest.yellowPixel.imx);
            telemetry.addData("avg_leftLeft",yellowTest.yellowPixel.avg_left);
            telemetry.addData("avg_leftRight",yellowTest.yellowPixel.avg_right);

            telemetry.update();
            sleep(100);
        }

        yellowTest.stopCamera();

    }
}
