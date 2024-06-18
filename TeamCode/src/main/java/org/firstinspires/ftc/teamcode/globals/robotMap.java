package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class robotMap {

    /**
     * Chassis
     */

    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;


 //   public MecanumDrivetrain drivetrain;

    /**
     * Intake
     */

    public DcMotorEx collect = null;
    public DcMotorEx extendoLeft = null;
    public DcMotorEx extendoRight = null;
    public Servo collectAngle = null;

    /**
     * Outtake
     */

    public DcMotorEx lift = null;
    public Servo fourbarLeft = null;
    public Servo fourbarRight = null;
    public Servo clawFlip = null;
    public Servo clawAngle = null;

    /**
     * Transfer
     */

    public Servo door = null;
    public Servo latchLeft = null;
    public Servo latchRight = null;

    /**
     * Sensors
     */

//    public DistanceSensor back = null;
//    public DistanceSensor extendoDistance = null;
    public DigitalChannel pixelLeft = null;
    public DigitalChannel pixelRight = null;
   // public DistanceSensor left = null;
   // public DistanceSensor right = null;

    /**
     * ENDGAME
     */

    public Servo pto = null;
    public Servo drone = null;
    public Servo droneLatch = null;


    /**
     * INIT
     */


    public robotMap(HardwareMap Init) {
        /**
         * Chassis
         */

        rightFront = Init.get(DcMotorEx.class, "rightFront");
        leftFront = Init.get(DcMotorEx.class, "leftFront");
        rightBack = Init.get(DcMotorEx.class, "rightBack");
        leftBack = Init.get(DcMotorEx.class, "leftBack");
      //  drivetrain = new MecanumDrivetrain();
        /**
         * Intake
         */

        collect = Init.get(DcMotorEx.class, "collect");
        collectAngle = Init.get(Servo.class, "collectAngle");
        extendoLeft = Init.get(DcMotorEx.class, "extendoLeft");
        extendoRight = Init.get(DcMotorEx.class, "extendoRight");

        /**
         * Outtake
         */

        lift = Init.get(DcMotorEx.class, "lift");
        fourbarLeft = Init.get(Servo.class, "fourbarLeft");
        fourbarRight = Init.get(Servo.class, "fourbarRight");
        clawFlip = Init.get(Servo.class, "clawFlip");
        clawAngle = Init.get(Servo.class, "clawAngle");

        /**
         * Transfer
         */

        door = Init.get(Servo.class, "door");
        latchLeft = Init.get(Servo.class, "latchLeft");
        latchRight = Init.get(Servo.class, "latchRight");


        /**
         * Sensors
         */

       // back = Init.get(DistanceSensor.class, "back");
       // extendoDistance = Init.get(DistanceSensor.class, "extendoDistance");
       // left = Init.get(DistanceSensor.class, "left");
       // right = Init.get(DistanceSensor.class, "right");
        pixelLeft = Init.get(DigitalChannel.class, "pixelLeft");
        pixelRight = Init.get(DigitalChannel.class, "pixelRight");

        /**
         * ENDGAME
         */

        pto = Init.get(Servo.class, "pto");
        drone = Init.get(Servo.class, "drone");
        droneLatch = Init.get(Servo.class, "droneLatch");

        /**
         * PROPERTIES
         *
         *
         *
         */

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        extendoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
       // collect.setDirection(DcMotorSimple.Direction.REVERSE);

//        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        DcMotor.RunMode prevMode = lift.getMode();
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(prevMode);

      //  lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  lift.setMode(DcMotor.RunMode.RUN_AND_RESET_ENCODER);






//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}
