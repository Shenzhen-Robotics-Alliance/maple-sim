package org.ironmaple.simulation.drivesims.configs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SwerveModuleConfig {
    public DCMotor driveMotor;
    public DCMotor steerMotor;
    public double driveCurrentLimit;
    public double driveGearRatio;
    public double steerGearRatio;
    public double driveFrictionVoltage;
    public double steerFrictionVoltage;
    public double tireCoefficientOfFriction;
    public double wheelsRadiusMeters;
    public double steerRotationalInertia;

    public SwerveModuleConfig(
            DCMotor driveMotor,
            DCMotor steerMotor,
            double driveCurrentLimit,
            double driveGearRatio,
            double steerGearRatio,
            double driveFrictionVoltage,
            double steerFrictionVoltage,
            double tireCoefficientOfFriction,
            double wheelsRadiusMeters,
            double steerRotationalInertia) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.driveCurrentLimit = driveCurrentLimit;
        this.driveGearRatio = driveGearRatio;
        this.steerGearRatio = steerGearRatio;
        this.driveFrictionVoltage = driveFrictionVoltage;
        this.steerFrictionVoltage = steerFrictionVoltage;
        this.tireCoefficientOfFriction = tireCoefficientOfFriction;
        this.wheelsRadiusMeters = wheelsRadiusMeters;
        this.steerRotationalInertia = steerRotationalInertia;
    }

    public SwerveModuleConfig() {
    }

    public SwerveModuleConfig withDriveMotor(DCMotor driveMotor) {
        this.driveMotor = driveMotor;
        return this;
    }

    public SwerveModuleConfig withSteerMotor(DCMotor steerMotor) {
        this.steerMotor = steerMotor;
        return this;
    }

    public SwerveModuleConfig withDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
        return this;
    }

    public SwerveModuleConfig withDriveGearRatio(double driveGearRatio) {
        this.driveGearRatio = driveGearRatio;
        return this;
    }

    public SwerveModuleConfig withSteerGearRatio(double steerGearRatio) {
        this.steerGearRatio = steerGearRatio;
        return this;
    }

    public SwerveModuleConfig withDriveFrictionVoltage(double driveFrictionVoltage) {
        this.driveFrictionVoltage = driveFrictionVoltage;
        return this;
    }

    public SwerveModuleConfig withSteerFrictionVoltage(double steerFrictionVoltage) {
        this.steerFrictionVoltage = steerFrictionVoltage;
        return this;
    }

    public SwerveModuleConfig withTireCoefficientOfFriction(double tireCoefficientOfFriction) {
        this.tireCoefficientOfFriction = tireCoefficientOfFriction;
        return this;
    }

    public SwerveModuleConfig withWheelsRadiusMeters(double wheelsRadiusMeters) {
        this.wheelsRadiusMeters = wheelsRadiusMeters;
        return this;
    }

    public SwerveModuleConfig withSteerRotationalInertia(double steerRotationalInertia) {
        this.steerRotationalInertia = steerRotationalInertia;
        return this;
    }

    
    /**
     *
     *
     * <h2>Stores the coefficient of friction of some common used wheels.</h2>
     */
    public enum WHEEL_GRIP {
        RUBBER_WHEEL(1.25),
        TIRE_WHEEL(1.2);

        public final double cof;

        WHEEL_GRIP(double cof) {
            this.cof = cof;
        }
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS Mark4
     * Swerve Module</a> for simulation
     */
    public static SwerveModuleConfig createMark4(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.8,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS
     * Mark4-i Swerve Module</a> for simulation
     */
    public static SwerveModuleConfig createMark4i(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.15;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                150.0 / 7.0,
                0.2,
                1,
                wheelCOF,
                Units.inchesToMeters(2),
                0.025);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS Mark4-n Swerve
     * Module</a> for simulation
     */
    public static SwerveModuleConfig createMark4n(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.13;
                    case 2 -> 5.9;
                    case 3 -> 5.36;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                18.75,
                0.25,
                1,
                wheelCOF,
                Units.inchesToMeters(2),
                0.025);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x">WCP SwerveX Swerve Module</a>
     * for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static SwerveModuleConfig createSwerveX(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.85;
                    case 2 -> 7.13;
                    case 3 -> 6.54;
                    case 4 -> 6.56;
                    case 5 -> 5.96;
                    case 6 -> 5.46;
                    case 7 -> 5.14;
                    case 8 -> 4.75;
                    case 9 -> 4.41;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                11.3142,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x-flipped">WCP SwerveX Flipped
     * Swerve Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static SwerveModuleConfig createSwerveXFlipped(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 8.1;
                    case 2 -> 7.36;
                    case 3 -> 6.75;
                    case 4 -> 6.72;
                    case 5 -> 6.11;
                    case 6 -> 5.6;
                    case 7 -> 5.51;
                    case 8 -> 5.01;
                    case 9 -> 4.59;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                11.3714,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP SwerveXS Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6
     */
    public static SwerveModuleConfig createSwerveXS(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.54;
                    case 3 -> 5.14;
                    case 4 -> 4.71;
                    case 5 -> 4.4;
                    case 6 -> 4.13;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                41.25,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(1.5),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2">WCP SwerveX2 Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9 <br>
     * X4 Ratios are gearRatioLevel 10-12
     */
    public static SwerveModuleConfig createSwerveX2(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 7.67;
                    case 2 -> 6.98;
                    case 3 -> 6.39;
                    case 4 -> 6.82;
                    case 5 -> 6.20;
                    case 6 -> 5.68;
                    case 7 -> 6.48;
                    case 8 -> 5.89;
                    case 9 -> 5.40;
                    case 10 -> 5.67;
                    case 11 -> 5.15;
                    case 12 -> 4.73;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.1,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(2),
                0.03);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP SwerveX2S Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static SwerveModuleConfig createSwerveX2S(
            DCMotor driveMotor, DCMotor steerMotor, double driveCurrentLimitAmps, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleConfig(
                driveMotor,
                steerMotor,
                driveCurrentLimitAmps,
                switch (gearRatioLevel) {
                    case 1 -> 6.0;
                    case 2 -> 5.63;
                    case 3 -> 5.29;
                    case 4 -> 4.94;
                    case 5 -> 4.67;
                    case 6 -> 4.42;
                    case 7 -> 4.11;
                    case 8 -> 3.9;
                    case 9 -> 3.71;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                25.9,
                0.2,
                0.3,
                wheelCOF,
                Units.inchesToMeters(1.875),
                0.03);
    }
}
