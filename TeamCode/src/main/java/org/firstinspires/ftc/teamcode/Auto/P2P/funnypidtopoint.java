//package org.firstinspires.ftc.teamcode.Auto.P2P;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
//
//import java.util.LinkedList;
//import java.util.List;
//
//@Config
//@Photon
//public class funnypidtopoint {
//    public SampleMecanumDrive drive;
//    private PIDFController pidX, pidY, pidHeading;
//    private LinkedList<Pose2d> targetPoses = new LinkedList<>();
//    private Pose2d targetPose;
//    private boolean sequenceFinished = false;
//    private ElapsedTime timer = new ElapsedTime();
//    private ElapsedTime stable = new ElapsedTime();
//    private ElapsedTime closeTimer = new ElapsedTime();
//    private boolean isRelativelyClose = false;
//
//    public static double KPX = 0.065, KDX = 0.012;
//    public static double KPY = 0.065, KDY = 0.012;
//    public static double KPANGLE = 0.7, KDANGLE = 0.02;
//
//    private final double MAX_TRANSLATIONAL_SPEED = 0.7;
//    private final double MAX_ROTATIONAL_SPEED = 0.83;
//    private static final double ALLOWED_TRANSLATIONAL_ERROR = 0.75;
//    private static final double ALLOWED_HEADING_ERROR = Math.toRadians(2);
//    private static final long STABLE_MS = 400;
//    private static final long DEAD_MS = 100;
//
//    private final long INTER_MAX_CLOSE_DURATION_MS = 0;
//
//    private final long FINAL_MAX_CLOSE_DURATION_MS = 1000;
//
//
//    public boolean hasReachedProximity;
//    public boolean reset_timer = true;
//
//    private static final double INTERMEDIATE_ALLOWED_TRANSLATIONAL_ERROR = 7;
//    private static final double INTERMEDIATE_ALLOWED_HEADING_ERROR = Math.toRadians(5);
//
//    private static final double INTERMEDIATE_ALLOWED_TRANSLATIONAL_ERROR2 = 3;
//    private static final double INTERMEDIATE_ALLOWED_HEADING_ERROR2 = Math.toRadians(5);
//
//
//    public funnypidtopoint(HardwareMap hardwareMap) {
//        drive = new SampleMecanumDrive(hardwareMap);
//
////        pidX = new PIDFController(0.1, 0, 0.035, 0);
////        pidY = new PIDFController(0.1, 0, 0.035, 0);
////        pidHeading = new PIDFController(0.9, 0, 0.045, 0);
//
//        pidX = new PIDFController(KPX, 0, KDX, 0);
//        pidY = new PIDFController(KPY, 0, KDY, 0);
//        pidHeading = new PIDFController(KPANGLE, 0, KDANGLE, 0);
//    }
//
//    public void setTargetPose(Pose2d targetPose) {
//        this.targetPose = targetPose;
//        hasReachedProximity = false;
//        reset_timer = true;
//        isRelativelyClose = false;
//    }
//
//    public void setTargetPoses(List<Pose> poses) {
//        targetPoses.clear();
//        targetPoses.addAll(poses);
//        sequenceFinished = targetPoses.isEmpty();
//        advanceToNextTarget();
//    }
//
//    private void advanceToNextTarget() {
//        if (!targetPoses.isEmpty()) {
//            setTargetPose(targetPoses.poll());
//        } else {
//            sequenceFinished = true;
//        }
//    }
//
//    public void update() {
//        if (!sequenceFinished && hasReachedTarget()) {
//            advanceToNextTarget();
//        }
//
//        if (targetPose == null) return;
//
//        Pose2d robotPose = drive.returnPose();
//        double[] motorPowers = getMotorPowers(robotPose, targetPose);
//        drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
//        long currentMaxCloseDurationMs = targetPoses.isEmpty() ? FINAL_MAX_CLOSE_DURATION_MS : INTER_MAX_CLOSE_DURATION_MS;
//
//        drive.update();
//
//        if (isCloseToTarget(robotPose, targetPose)) {
//            if (reset_timer) {
//                stable.reset();
//                reset_timer = false;
//            }
//            if (stable.milliseconds() > STABLE_MS) {
//                hasReachedProximity = true;
//                reset_timer = true;
//                isRelativelyClose = false;
//                closeTimer.reset();
//            }
//        } else {
//            if (isRelativelyCloseToTarget(robotPose, targetPose)) {
//                if (!isRelativelyClose) {
//                    closeTimer.reset();
//                    isRelativelyClose = true;
//                } else if (closeTimer.milliseconds() > currentMaxCloseDurationMs) {
//                    hasReachedProximity = true;
//                    advanceToNextTarget();
//                    isRelativelyClose = false;
//                }
//            }
//
//            if (!isRelativelyCloseToTarget(robotPose, targetPose)) {
//                isRelativelyClose = false;
//            }
//        }
//    }
//
//    private boolean isCloseToTarget(Pose robotPose, Pose targetPose) {
//        Pose delta = targetPose.subtract(robotPose);
//        return delta.toVec2D().magnitude() <= (ALLOWED_TRANSLATIONAL_ERROR) &&
//                Math.abs(delta.getY()) <= (ALLOWED_TRANSLATIONAL_ERROR) &&
//                Math.abs(delta.heading) <= (ALLOWED_HEADING_ERROR);
//    }
//
//    private boolean isRelativelyCloseToTarget(Pose robotPose, Pose targetPose) {
//
//        double allowedTranslationalError = targetPoses.isEmpty() ? INTERMEDIATE_ALLOWED_TRANSLATIONAL_ERROR2 : INTERMEDIATE_ALLOWED_TRANSLATIONAL_ERROR;
//        double allowedHeadingError = targetPoses.isEmpty() ? INTERMEDIATE_ALLOWED_HEADING_ERROR2 : INTERMEDIATE_ALLOWED_HEADING_ERROR;
//
//        Pose delta = targetPose.subtract(robotPose);
//        return delta.toVec2D().magnitude() <= allowedTranslationalError &&
//                Math.abs(delta.getY()) <= allowedTranslationalError &&
//                Math.abs(delta.heading) <= allowedHeadingError;
//    }
//
//    public boolean hasReachedTarget() {
//        return hasReachedProximity;
//    }
//
//    public boolean isSequenceFinished() {
//        return sequenceFinished;
//    }
//
//    public double[] getMotorPowers(Pose robotPose, Pose targetPose) {
//
//        double headingDifference = targetPose.heading - robotPose.heading;
//        if (headingDifference > Math.PI) targetPose.heading -= 2 * Math.PI;
//        if (headingDifference < -Math.PI) targetPose.heading += 2 * Math.PI;
//
//        double xPower = pidX.calculate(robotPose.getX(), targetPose.getX());
//        double yPower = -pidY.calculate(robotPose.getY(), targetPose.getY());
//        double hPower = -pidHeading.calculate(robotPose.heading, targetPose.heading);
//
//        double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
//        double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);
//
//        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
//        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
//        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
//
//        double frontLeftPower = x_rotated + y_rotated + hPower;
//        double backLeftPower = x_rotated - y_rotated + hPower;
//        double backRightPower = x_rotated + y_rotated - hPower;
//        double frontRightPower = x_rotated - y_rotated - hPower;
//
//        double maxPower = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
//        frontLeftPower /= maxPower;
//        backLeftPower /= maxPower;
//        backRightPower /= maxPower;
//        frontRightPower /= maxPower;
//
//        return new double[] {frontLeftPower, backLeftPower, backRightPower, frontRightPower};
//    }
//
//    public void stopMotors() {
//        drive.setMotorPowers(0, 0, 0, 0);
//    }
//}
