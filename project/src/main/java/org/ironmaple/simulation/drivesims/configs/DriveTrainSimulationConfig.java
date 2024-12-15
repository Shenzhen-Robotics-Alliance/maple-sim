package org.ironmaple.simulation.drivesims.configs;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 *
 *
 * <h1>Stores the configurations for a swerve drive simulation.</h1>
 *
 * <p>This class is used to hold all the parameters necessary for simulating a swerve drivetrain, allowing for realistic
 * performance testing and evaluation.
 */
public class DriveTrainSimulationConfig {
    public Mass robotMass;
    public Distance bumperLengthX, bumperWidthY;
    public Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory;
    public Supplier<GyroSimulation> gyroSimulationFactory;
    public Translation2d[] moduleTranslations;

    /**
     *
     *
     * <h2>Ordinary Constructor</h2>
     *
     * <p>Creates an instance of {@link DriveTrainSimulationConfig} with specified parameters.
     *
     * @param robotMass the mass of the robot, including bumpers.
     * @param bumperLengthX the length of the bumper (distance from front to back).
     * @param bumperWidthY the width of the bumper (distance from left to right).
     * @param trackLengthX the distance between the front and rear wheels.
     * @param trackWidthY the distance between the left and right wheels.
     * @param swerveModuleSimulationFactory the factory that creates appropriate swerve module simulation for the
     *     drivetrain.
     * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the drivetrain.
     */
    public DriveTrainSimulationConfig(
            Mass robotMass,
            Distance bumperLengthX,
            Distance bumperWidthY,
            Distance trackLengthX,
            Distance trackWidthY,
            Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory,
            Supplier<GyroSimulation> gyroSimulationFactory) {
        this.robotMass = robotMass;
        this.bumperLengthX = bumperLengthX;
        this.bumperWidthY = bumperWidthY;

        this.swerveModuleSimulationFactory = swerveModuleSimulationFactory;
        this.gyroSimulationFactory = gyroSimulationFactory;
        this.withTrackLengthTrackWidth(trackLengthX, trackWidthY);

        checkRobotMass();
        checkBumperSize();
    }

    /**
     *
     *
     * <h2>Default Constructor.</h2>
     *
     * <p>Creates a {@link DriveTrainSimulationConfig} with all the data set to default values.
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
     * @return a new instance of {@link DriveTrainSimulationConfig} with all configs set to default values.
     */
    public static DriveTrainSimulationConfig Default() {
        return new DriveTrainSimulationConfig(
                Kilograms.of(45),
                Meters.of(0.76),
                Meters.of(.76),
                Meters.of(0.52),
                Meters.of(0.52),
                COTS.ofMark4(DCMotor.getFalcon500(1), DCMotor.getFalcon500(1), COTS.WHEELS.COLSONS.cof, 2),
                COTS.ofPigeon2());
    }

    /**
     *
     *
     * <h2>Sets the robot mass.</h2>
     *
     * <p>Updates the mass of the robot in kilograms.
     *
     * @param robotMass the new mass of the robot.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    public DriveTrainSimulationConfig withRobotMass(Mass robotMass) {
        this.robotMass = robotMass;
        checkRobotMass();
        return this;
    }

    /**
     *
     *
     * <h2>Sets the bumper size.</h2>
     *
     * <p>Updates the dimensions of the bumper.
     *
     * @param bumperLengthX the length of the bumper.
     * @param bumperWidthY the width of the bumper.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    public DriveTrainSimulationConfig withBumperSize(Distance bumperLengthX, Distance bumperWidthY) {
        this.bumperLengthX = bumperLengthX;
        this.bumperWidthY = bumperWidthY;

        checkBumperSize();
        return this;
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
     * @param trackLengthX the distance between the front and rear wheels.
     * @param trackWidthY the distance between the left and right wheels.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    public DriveTrainSimulationConfig withTrackLengthTrackWidth(Distance trackLengthX, Distance trackWidthY) {
        BoundingCheck.check(trackLengthX.in(Meters), 0.5, 1.5, "track length", "meters");
        BoundingCheck.check(trackWidthY.in(Meters), 0.5, 1.5, "track width", "meters");

        this.moduleTranslations = new Translation2d[] {
            new Translation2d(trackLengthX.in(Meters) / 2, trackWidthY.in(Meters) / 2),
            new Translation2d(trackLengthX.in(Meters) / 2, -trackWidthY.in(Meters) / 2),
            new Translation2d(-trackLengthX.in(Meters) / 2, trackWidthY.in(Meters) / 2),
            new Translation2d(-trackLengthX.in(Meters) / 2, -trackWidthY.in(Meters) / 2)
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
     * <p>For ordinary rectangular modules configuration, use {@link #withTrackLengthTrackWidth(Distance, Distance)}
     * instead.
     *
     * @param moduleTranslations the custom translations for the swerve modules.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    public DriveTrainSimulationConfig withCustomModuleTranslations(Translation2d[] moduleTranslations) {
        checkModuleTranslations();
        this.moduleTranslations = moduleTranslations;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the swerve module simulation factory.</h2>
     *
     * <p>Updates the factory used to create swerve module simulations.
     *
     * @param swerveModuleSimulationFactory the new factory for swerve module simulations.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    public DriveTrainSimulationConfig withSwerveModule(Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory) {
        this.swerveModuleSimulationFactory = swerveModuleSimulationFactory;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the gyro simulation factory.</h2>
     *
     * <p>Updates the factory used to create gyro simulations.
     *
     * @param gyroSimulationFactory the new factory for gyro simulations.
     * @return the current instance of {@link DriveTrainSimulationConfig} for method chaining.
     */
    public DriveTrainSimulationConfig withGyro(Supplier<GyroSimulation> gyroSimulationFactory) {
        this.gyroSimulationFactory = gyroSimulationFactory;
        return this;
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
    public double getDensityKgPerSquaredMeters() {
        return robotMass.in(Kilograms) / (bumperLengthX.in(Meters) * bumperWidthY.in(Meters));
    }

    /**
     *
     *
     * <h2>Calculates the track length in the X direction.</h2>
     *
     * <p>Returns the total distance between the frontmost and rearmost module translations in the X direction.
     *
     * @return the track length.
     * @throws IllegalStateException if the module translations are empty.
     */
    public Distance trackLengthX() {
        final OptionalDouble maxModuleX = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getX)
                .max();
        final OptionalDouble minModuleX = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getX)
                .min();
        if (maxModuleX.isEmpty() || minModuleX.isEmpty())
            throw new IllegalStateException("Modules translations are empty");
        return Meters.of(maxModuleX.getAsDouble() - minModuleX.getAsDouble());
    }

    /**
     *
     *
     * <h2>Calculates the track width in the Y direction.</h2>
     *
     * <p>Returns the total distance between the leftmost and rightmost module translations in the Y direction.
     *
     * @return the track width.
     * @throws IllegalStateException if the module translations are empty.
     */
    public Distance trackWidthY() {
        final OptionalDouble maxModuleY = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getY)
                .max();
        final OptionalDouble minModuleY = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getY)
                .min();
        if (maxModuleY.isEmpty() || minModuleY.isEmpty())
            throw new IllegalStateException("Modules translations are empty");
        return Meters.of(maxModuleY.getAsDouble() - minModuleY.getAsDouble());
    }

    public Distance driveBaseRadius() {
        return Meters.of(Math.hypot(trackLengthX().in(Meters), trackWidthY().in(Meters)));
    }

    private void checkRobotMass() {
        BoundingCheck.check(robotMass.in(Kilograms), 10, 80, "robot mass", "kg");
    }

    private void checkBumperSize() {
        BoundingCheck.check(bumperLengthX.in(Meters), 0.5, 1.5, "bumper length", "meters");
        BoundingCheck.check(bumperWidthY.in(Meters), 0.5, 1.5, "bumper width", "meters");
    }

    private void checkModuleTranslations() {
        for (int i = 0; i < moduleTranslations.length; i++)
            BoundingCheck.check(
                    moduleTranslations[i].getNorm(),
                    0.2,
                    1.2,
                    "module number " + i + " translation magnitude",
                    "meters");
    }
}
