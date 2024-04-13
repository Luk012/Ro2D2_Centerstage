package org.firstinspires.ftc.teamcode;

public class DistanceSensorCalibrator {
    private double[] rawReadings;
    private double[] actualDistances;
    private double[] coefficients; // Coefficients of the calibration curve

    public DistanceSensorCalibrator(double[] rawReadings, double[] actualDistances) {
        this.rawReadings = rawReadings;
        this.actualDistances = actualDistances;
        // Fit calibration curve to raw readings and actual distances
        this.coefficients = fitCalibrationCurve(rawReadings, actualDistances);
    }

    public double calibrate(double rawReading) {
        // Evaluate calibration curve at the raw reading to obtain calibrated distance
        return evaluateCalibrationCurve(rawReading, coefficients);
    }

    private double[] fitCalibrationCurve(double[] rawReadings, double[] actualDistances) {

        double[] coefficients = {0.0, 0.0};

        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumXX = 0.0;
        int n = rawReadings.length;
        for (int i = 0; i < n; i++) {
            sumX += rawReadings[i];
            sumY += actualDistances[i];
            sumXY += rawReadings[i] * actualDistances[i];
            sumXX += rawReadings[i] * rawReadings[i];
        }
        double slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
        double intercept = (sumY - slope * sumX) / n;
        coefficients[0] = slope;
        coefficients[1] = intercept;
        return coefficients;
    }

    private double evaluateCalibrationCurve(double rawReading, double[] coefficients) {

        double distance = coefficients[0] * rawReading + coefficients[1];
        return distance;
    }
}
