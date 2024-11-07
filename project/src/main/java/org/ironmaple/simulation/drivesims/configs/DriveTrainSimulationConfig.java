package org.ironmaple.simulation.drivesims.configs;

import org.ironmaple.simulation.drivesims.DriveTrainSimulation;

/**
 *
 *
 * <h1>Stores the configurations for a swerve drive simulation.</h1>
 *
 * <p>This class is used to hold all the parameters necessary for simulating a drive drivetrain, allowing for realistic
 * performance testing and evaluation.
 */
public class DriveTrainSimulationConfig<T extends DriveTrainSimulation, S extends DriveTrainSimulationConfig<T, S>> {
    public double robotMassKg, bumperLengthXMeters, bumperWidthYMeters;

    /**
     *
     *
     * <h2>Ordinary Constructor</h2>
     *
     * <p>Creates an instance of {@link DriveTrainSimulationConfig} with specified parameters.
     *
     * @param robotMassKg the mass of the robot in kilograms, including bumpers.
     * @param bumperLengthXMeters the length of the bumper in meters (distance from front to back).
     * @param bumperWidthYMeters the width of the bumper in meters (distance from left to right).
     */
    protected DriveTrainSimulationConfig(
            double robotMassKg,
            double bumperLengthXMeters,
            double bumperWidthYMeters) {
        this.robotMassKg = robotMassKg;
        this.bumperLengthXMeters = bumperLengthXMeters;
        this.bumperWidthYMeters = bumperWidthYMeters;
    }

    /**
     *
     *
     * <h2>Sets the robot mass.</h2>
     *
     * <p>Updates the mass of the robot in kilograms.
     *
     * @param robotMassKg the new mass of the robot in kilograms.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    @SuppressWarnings("unchecked")
    public S withRobotMass(double robotMassKg) {
        this.robotMassKg = robotMassKg;
        return (S) this;
    }

    /**
     *
     *
     * <h2>Sets the bumper size.</h2>
     *
     * <p>Updates the dimensions of the bumper.
     *
     * @param bumperLengthXMeters the length of the bumper in meters.
     * @param bumperWidthYMeters the width of the bumper in meters.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    @SuppressWarnings("unchecked")
    public S withBumperSize(double bumperLengthXMeters, double bumperWidthYMeters) {
        this.bumperLengthXMeters = bumperLengthXMeters;
        this.bumperWidthYMeters = bumperWidthYMeters;
        return (S) this;
    }

    /**
     *
     *
     * <h2>Calculates the density of the robot.</h2>
     *
     * <p>Returns the density of the robot based on its mass and bumper dimensions.
     *
     * @return the density in kilograms per square meter.
     */
    public double getDensity() {
        return robotMassKg / (bumperLengthXMeters * bumperWidthYMeters);
    }
}
