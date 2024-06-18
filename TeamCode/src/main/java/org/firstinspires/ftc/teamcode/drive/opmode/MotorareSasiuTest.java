package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
    @TeleOp(name = "MotoareSasiuTest")
public class MotorareSasiuTest extends OpMode {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;


    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            leftFront.setPower(1);
        } else {
            leftFront.setPower(0);
        }

        if (gamepad1.b) {
            rightFront.setPower(1);
        } else {
            rightFront.setPower(0);
        }

        if (gamepad1.x) {
            leftBack.setPower(1);
        } else {
            leftBack.setPower(0);
        }

        if (gamepad1.y) {
            rightBack.setPower(1);
        } else {
            rightBack.setPower(0);
        }


        telemetry.addData("leftback", leftBack.getCurrentPosition());
        telemetry.addData("leftfront", leftFront.getCurrentPosition());
        telemetry.addData("rightback", rightBack.getCurrentPosition());
        telemetry.addData("rightfront", rightFront.getCurrentPosition());

        telemetry.update();
    }
}
