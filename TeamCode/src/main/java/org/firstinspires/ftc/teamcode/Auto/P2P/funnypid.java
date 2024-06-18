//package org.firstinspires.ftc.teamcode.Auto.P2P;
//
//public class funnypid {
//    private double p, i, d;
//    private double integral = 0;
//    private double previousError = 0;
//    private double derivative = 0;
//    private double setpoint = 0;
//    private double outputMin = Double.NEGATIVE_INFINITY;
//    private double outputMax = Double.POSITIVE_INFINITY;
//    private double maxIntegral = 1000;
//    private double derivativeSmoothingFactor = 0.5;
//    private double lastDerivative = 0;
//    private double feedForward = 0;
//
//    public funnypid(double p, double i, double d) {
//        this.p = p;
//        this.i = i;
//        this.d = d;
//    }
//
//    public void setSetpoint(double setpoint) {
//        this.setpoint = setpoint;
//        this.integral = 0;
//        this.previousError = 0;
//        this.derivative = 0;
//    }
//
//    public void setOutputLimits(double min, double max) {
//        this.outputMin = min;
//        this.outputMax = max;
//    }
//
//    public void setMaxIntegral(double maxIntegral) {
//        this.maxIntegral = maxIntegral;
//    }
//
//    public void setDerivativeSmoothingFactor(double factor) {
//        this.derivativeSmoothingFactor = factor;
//    }
//
//    public void setFeedForward(double feedForward) {
//        this.feedForward = feedForward;
//    }
//
//    public double calculate(double currentPosition) {
//        double error = setpoint - currentPosition;
//
//        integral += error;
//        integral = clamp(integral, -maxIntegral, maxIntegral);
//
//        double deltaError = error - previousError;
//
//        derivative = (derivativeSmoothingFactor * deltaError) + ((1 - derivativeSmoothingFactor) * lastDerivative);
//        lastDerivative = derivative;
//
//        previousError = error;
//
//        double output = (p * error) + (i * integral) + (d * derivative) + feedForward;
//
//        return clamp(output, outputMin, outputMax);
//    }
//
//    public void reset() {
//        integral = 0;
//        previousError = 0;
//        derivative = 0;
//        lastDerivative = 0;
//    }
//
//    private double clamp(double value, double min, double max) {
//        return Math.max(min, Math.min(max, value));
//    }
//}
