package org.ironmaple.simulation.drivesims.configs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Arrays;
import java.util.OptionalDouble;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 *
 *
 * <h1>Stores the configurations for a swerve drive simulation.</h1>
 *
 * <p>This class is used to hold all the parameters necessary for simulating a swerve drivetrain, allowing for realistic
 * performance testing and evaluation.
 */
public class SwerveConfig extends DriveTrainSimulationConfig<SwerveDriveSimulation, SwerveConfig> {
    public SwerveModuleConfig swerveModuleConfig;
    public GyroConfig gyroConfig;
    public Translation2d[] moduleTranslations;

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
     * @param trackLengthXMeters the distance between the front and rear wheels, in meters.
     * @param trackWidthYMeters the distance between the left and right wheels, in meters.
     * @param swerveModuleSimulationFactory the factory that creates appropriate swerve module simulation for the
     *     drivetrain.
     * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the drivetrain.
     */
    public SwerveConfig(
            double robotMassKg,
            double bumperLengthXMeters,
            double bumperWidthYMeters,
            double trackLengthXMeters,
            double trackWidthYMeters,
            SwerveModuleConfig swerveModuleConfig,
            GyroConfig gyroConfig) {
        super(robotMassKg, bumperLengthXMeters, bumperWidthYMeters);

        this.swerveModuleConfig = swerveModuleConfig;
        this.gyroConfig = gyroConfig;
        this.withTrackLengthTrackWidth(trackLengthXMeters, trackWidthYMeters);
    }

    /**
     *
     *
     * <h2>Default Constructor.</h2>
     *
     * <p>Creates a {@link SwerveConfig} with all the data set to default values.
     *
     * <p>Though the config starts with default values, any configuration can be modified after creation.
     *
     * <p>The default configurations are:
     *
     * <ul>
     *   <li>Robot Mass of 45 kilograms.
     *   <li>Bumper Length of 0.76 meters.
     *   <li>Bumper Width of 0.76 meters.
     *   <li>Track Length of 0.52 meters.
     *   <li>Track Width of 0.52 meters.
     *   <li>Default swerve module simulations based on Falcon 500 motors.
     *   <li>Default gyro simulation using the Pigeon2 gyro.
     * </ul>
     *
     * @return a new instance of {@link SwerveConfig} with all configs set to default values.
     */
    public static SwerveConfig Default() {
        return new SwerveConfig(
                45,
                0.76,
                0.76,
                0.52,
                0.52,
                SwerveModuleConfig.createMark4(
                        DCMotor.getFalcon500(1),
                        DCMotor.getFalcon500(1),
                        60,
                        SwerveModuleConfig.WHEEL_GRIP.RUBBER_WHEEL.cof,
                        2),
                GyroConfig.createPigeon2());
    }

    /**
     *
     *
     * <h2>Sets the track length and width.</h2>
     *
     * <p>Updates the translations for the swerve modules based on the specified track length and track width.
     *
     * <p>For non-rectangular chassis configuration, use {@link #withCustomModuleTranslations(Translation2d[])} instead.
     *
     * @param trackLengthXMeters the distance between the front and rear wheels, in meters.
     * @param trackWidthYMeters the distance between the left and right wheels, in meters.
     * @return the current instance of {@link SwerveConfig} for method chaining.
     */
    public SwerveConfig withTrackLengthTrackWidth(double trackLengthXMeters, double trackWidthYMeters) {
        this.moduleTranslations = new Translation2d[] {
            new Translation2d(trackLengthXMeters / 2, trackWidthYMeters / 2),
            new Translation2d(trackLengthXMeters / 2, -trackWidthYMeters / 2),
            new Translation2d(-trackLengthXMeters / 2, trackWidthYMeters / 2),
            new Translation2d(-trackLengthXMeters / 2, -trackWidthYMeters / 2)
        };
        return this;
    }

    /**
     *
     *
     * <h2>Sets custom module translations.</h2>
     *
     * <p>Updates the translations of the swerve modules with user-defined values.
     *
     * <p>For ordinary rectangular modules configuration, use {@link #withTrackLengthTrackWidth(double, double)}
     * instead.
     *
     * @param moduleTranslations the custom translations for the swerve modules.
     * @return the current instance of {@link SwerveConfig} for method chaining.
     */
    public SwerveConfig withCustomModuleTranslations(Translation2d[] moduleTranslations) {
        this.moduleTranslations = moduleTranslations;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the swerve module config.</h2>
     *
     * @param swerveModuleConfig the new config for swerve module simulations.
     * @return the current instance of {@link SwerveConfig} for method chaining.
     */
    public SwerveConfig withSwerveModule(SwerveModuleConfig swerveModuleConfig) {
        this.swerveModuleConfig = swerveModuleConfig;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the gyro config.</h2>
     *
     * @param gyroConfig the new config for gyro simulation.
     * @return the current instance of {@link SwerveConfig} for method chaining.
     */
    public SwerveConfig withGyro(GyroConfig gyroConfig) {
        this.gyroConfig = gyroConfig;
        return this;
    }

    /**
     *
     *
     * <h2>Calculates the track length in the X direction.</h2>
     *
     * <p>Returns the total distance between the frontmost and rearmost module translations in the X direction.
     *
     * @return the track length in meters.
     * @throws IllegalStateException if the module translations are empty.
     */
    public double getTrackLengthX() {
        final OptionalDouble maxModuleX = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getX)
                .max();
        final OptionalDouble minModuleX = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getX)
                .min();
        if (maxModuleX.isEmpty() || minModuleX.isEmpty())
            throw new IllegalStateException("Modules translations are empty");
        return maxModuleX.getAsDouble() - minModuleX.getAsDouble();
    }

    /**
     *
     *
     * <h2>Calculates the track width in the Y direction.</h2>
     *
     * <p>Returns the total distance between the leftmost and rightmost module translations in the Y direction.
     *
     * @return the track width in meters.
     * @throws IllegalStateException if the module translations are empty.
     */
    public double getTrackWidthY() {
        final OptionalDouble maxModuleY = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getY)
                .max();
        final OptionalDouble minModuleY = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getY)
                .min();
        if (maxModuleY.isEmpty() || minModuleY.isEmpty())
            throw new IllegalStateException("Modules translations are empty");
        return maxModuleY.getAsDouble() - minModuleY.getAsDouble();
    }
}
