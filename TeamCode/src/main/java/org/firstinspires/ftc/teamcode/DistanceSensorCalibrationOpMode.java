package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@TeleOp(name = "Distance Sensor Calibration", group = "OpMode")
public class DistanceSensorCalibrationOpMode extends LinearOpMode {

    private DistanceSensorCalibrator calibrator;

    @Override
    public void runOpMode() throws InterruptedException {

        robotMap r = new robotMap(hardwareMap);
        //LowPassFilter l = new LowPassFilter(0.9, r.right.getDistance(DistanceUnit.CM));

        // Wait for start button
        waitForStart();

        // Create arrays to store calibration data



        double[] rawReadingsExtendo = {36.5, 33.1, 34.6, 34.4, 36.5, 37.1, 38.6, 30.8, 23.3, 40.2, 25.5, 27.8, 28.9, 29.6, 27.6, 26.3, 24.9, 36.8, 35.6, 37.3};
        double[] actualDistancesExtendo = {27, 25, 26, 25.5, 26.5, 27.5, 29, 24, 20, 30, 21, 22, 23, 23.5, 22.5, 21.5, 20, 28, 28.5, 29.5};


        /* SENZOR SPATE */
//        double[] rawReadings = {28.5, 27.3, 26.2, 24.9, 24.3, 23.6, 28.4, 29.5, 30.6, 31.2, 31.3, 31.6, 32.3, 33.4, 34.5, 35.3, 36.1, 37.8, 37.9, 38};
//        double[] actualDistances = {23, 22, 21.5, 21, 20.5, 20, 22.5, 24, 24.4, 24.6, 25.2, 25.5, 26, 26.6, 27.2, 28.1, 28.9, 29.6, 30, 30.5};

        // Perform calibration using collected data
        calibrator = new DistanceSensorCalibrator(rawReadingsExtendo, actualDistancesExtendo);

        while (opModeIsActive() && !isStopRequested()) {
            // Get raw sensor reading
           // double rawReading = r.extendoDistance.getDistance(DistanceUnit.CM);

//            // Calibrate raw reading
//            double calibratedDistance = calibrator.calibrate(rawReading);
//
//            // Display raw and calibrated readings
//            telemetry.addData("Raw Reading", rawReading);
//            //telemetry.addData("Calibrated Distance WITH Filter", l.getValue(calibratedDistance));
//            telemetry.addData("Calibrated Distance WITHOUT Filter", calibratedDistance);
//            telemetry.addData("Difference", calibratedDistance - rawReading);
            telemetry.update();
        }
    }
}
