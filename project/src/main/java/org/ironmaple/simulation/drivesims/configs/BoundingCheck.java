package org.ironmaple.simulation.drivesims.configs;

import edu.wpi.first.wpilibj.DriverStation;

public class BoundingCheck {
    public static void check(double value, double lowerBound, double upperBound, String variableName, String unit) {
        if (lowerBound <= value && value <= upperBound) return;
        final String errorMessage = "The provided \"" + variableName + "\" is " + value + unit
                + ", which seems abnormal, please check its correctness";
        DriverStation.reportError(errorMessage, true);
    }
}
