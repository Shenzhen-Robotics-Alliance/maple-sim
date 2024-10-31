package org.ironmaple.simulation.drivesims.configs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.Supplier;

public class DriveTrainSimulationConfig {
    public double robotMassKg, bumperLengthXMeters, bumperWidthYMeters;
    public Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory;
    public Supplier<GyroSimulation> gyroSimulationFactory;
    public Translation2d[] moduleTranslations;

    public DriveTrainSimulationConfig(double robotMassKg, double bumperLengthXMeters, double bumperWidthYMeters, double trackLengthXMeters, double trackWidthYMeters, Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory, Supplier<GyroSimulation> gyroSimulationFactory) {
        this.robotMassKg = robotMassKg;
        this.bumperLengthXMeters = bumperLengthXMeters;
        this.bumperWidthYMeters = bumperWidthYMeters;

        this.swerveModuleSimulationFactory = swerveModuleSimulationFactory;
        this.gyroSimulationFactory = gyroSimulationFactory;
        this.withTrackLengthTrackWidth(trackLengthXMeters, trackWidthYMeters);
    }

    public static DriveTrainSimulationConfig Default() {
        return new DriveTrainSimulationConfig(
                45,
                0.76, 0.76,
                0.52, 0.52,
                SwerveModuleSimulation.getMark4(
                        DCMotor.getFalcon500(1),
                        DCMotor.getFalcon500(1),
                        60,
                        SwerveModuleSimulation.DRIVE_WHEEL_TYPE.RUBBER,
                        2
                ),
                GyroSimulation.getPigeon2()
        );
    }

    public DriveTrainSimulationConfig withRobotMass(double robotMassKg) {
        this.robotMassKg = robotMassKg;
        return this;
    }

    public DriveTrainSimulationConfig withBumperSize(double bumperLengthXMeters, double bumperWidthYMeters) {
        this.bumperLengthXMeters = bumperLengthXMeters;
        this.bumperWidthYMeters = bumperWidthYMeters;
        return this;
    }

    public DriveTrainSimulationConfig withTrackLengthTrackWidth(double trackLengthXMeters, double trackWidthYMeters) {
        this.moduleTranslations = new Translation2d[]{
                new Translation2d(trackLengthXMeters/2, trackWidthYMeters / 2),
                new Translation2d(trackLengthXMeters/2, -trackWidthYMeters / 2),
                new Translation2d(-trackLengthXMeters/2, trackWidthYMeters / 2),
                new Translation2d(-trackLengthXMeters/2, -trackWidthYMeters / 2)
        };
        return this;
    }

    public DriveTrainSimulationConfig withCustomModuleTranslations(Translation2d[] moduleTranslations) {
        this.moduleTranslations = moduleTranslations;
        return this;
    }

    public DriveTrainSimulationConfig withSwerveModule(Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory) {
        this.swerveModuleSimulationFactory = swerveModuleSimulationFactory;
        return this;
    }

    public DriveTrainSimulationConfig withGyro(Supplier<GyroSimulation> gyroSimulationFactory) {
        this.gyroSimulationFactory = gyroSimulationFactory;
        return this;
    }

    public double getDensity() {
        return robotMassKg / (bumperLengthXMeters * bumperWidthYMeters);
    }

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
