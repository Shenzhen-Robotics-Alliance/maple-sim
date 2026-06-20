package org.ironmaple.simulation.drivesims.configs;

import org.wpilib.driverstation.DriverStationErrors;

public class BoundingCheck {
    public static void check(double value, double lowerBound, double upperBound, String variableName, String unit) {
        if (lowerBound <= value && value <= upperBound) return;
        final String errorMessage = "The provided \"" + variableName + "\" is " + value + unit
                + ", which seems abnormal, please check its correctness";
        DriverStationErrors.reportError(errorMessage, true);
    }
}
