package org.firstinspires.ftc.teamcode.drive.opmode;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.Recognition.AtagMath;
import org.firstinspires.ftc.teamcode.Auto.Recognition.AtagYellow;
import org.firstinspires.ftc.teamcode.Auto.Recognition.YellowPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "analogbby", group = "Concept")
public class analogbby extends LinearOpMode {

    private AnalogInput analogSensor;

    @Override
    public void runOpMode() {

        analogSensor = hardwareMap.get(AnalogInput.class, "analog0");

        waitForStart();

        while (opModeIsActive()) {

            double analogValue = analogSensor.getVoltage();

            telemetry.addData("analog value", analogValue);
        //    telemetry.addData("distance", (analogValue / (5.0 / 1024.0)) * 6.0 - 300);
            telemetry.addData("distance", (analogValue * 1447.07) - 305.685);
            telemetry.update();

        }

    }
}