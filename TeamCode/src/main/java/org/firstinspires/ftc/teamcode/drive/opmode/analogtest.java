//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AnalogTest", group = "TeleOp")
//public class analogtest extends OpMode {
//
//    private AnalogInput analogSensor;
//
//    @Override
//    public void init() {
//        analogSensor = hardwareMap.get(AnalogInput.class, "analog0");
//    }
//
//    @Override
//    public void loop() {
//        // Citiți valoarea senzorului analogic
//        double analogValue = analogSensor.getVoltage();
//
//        // Afișați valoarea citită în telemetrie
//        telemetry.addData("Analog Value", analogValue);
//        telemetry.update();
//    }
//}
//
