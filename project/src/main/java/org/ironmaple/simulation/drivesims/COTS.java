package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.Supplier;

public class COTS {
    /**
     *
     *
     * <h2>Stores the coefficient of friction of some common used wheels.</h2>
     *
     * <p>Data comes from <a href='https://www.chiefdelphi.com/t/spectrum-3847-build-blog-2024/447471/217'>Spectrum
     * 3847's Build Blog</a>.
     */
    public enum WHEELS {
        /** <a href='https://www.vexrobotics.com/colsonperforma.html'>Colsons Wheels.</a> */
        COLSONS(0.899),
        /**
         * Default Neoprene Treads for <a
         * href='https://www.swervedrivespecialties.com/products/mk4i-swerve-module'>Mark4 Modules</a>
         */
        DEFAULT_NEOPRENE_TREAD(1.426),
        /**
         * <a href='https://www.andymark.com/products/blue-nitrile-roughtop-tread-1-in-wide-10-ft-long'>Blue Nitrile
         * Tread from AndyMark.</a>
         */
        BLUE_NITRILE_TREAD(1.542),
        /** <a href='https://www.vexrobotics.com/217-9064.html'>Vex Grip V2 Wheel.</a> */
        VEX_GRIP_V2(1.916),
        /**
         * <a href='https://www.thebluealliance.com/team/88'>Team 88</a>'s <a
         * href='https://www.chiefdelphi.com/t/tpu90a-grippy-tire-cad-published-finally/438075'>TPU90A Grippy Tire</a>
         */
        SLS_PRINTED_WHEELS(2.106);

        public final double cof;

        WHEELS(double cof) {
            this.cof = cof;
        }
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS Mark4
     * Swerve Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> ofMark4(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.8,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module">SDS
     * Mark4-i Swerve Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> ofMark4i(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.15;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                150.0 / 7.0,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS Mark4-n Swerve
     * Module</a> for simulation
     */
    public static Supplier<SwerveModuleSimulation> ofMark4n(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 7.13;
                    case 2 -> 5.9;
                    case 3 -> 5.36;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                18.75,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x">WCP SwerveX Swerve Module</a>
     * for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> ofSwerveX(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
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
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x-flipped">WCP SwerveX Flipped
     * Swerve Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> ofSwerveXFlipped(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
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
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP SwerveXS Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6
     */
    public static Supplier<SwerveModuleSimulation> ofSwerveXS(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
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
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
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
    public static Supplier<SwerveModuleSimulation> ofSwerveX2(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
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
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP SwerveX2S Swerve
     * Module</a> for simulation
     *
     * <p>X1 Ratios are gearRatioLevel 1-3 <br>
     * X2 Ratios are gearRatioLevel 4-6 <br>
     * X3 Ratios are gearRatioLevel 7-9
     */
    public static Supplier<SwerveModuleSimulation> ofSwerveX2S(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
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
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * Creates a <a href="https://www.revrobotics.com/rev-21-3005/">REV MAXSwerve swerve module</a> for simulation
     *
     * <p>Base Kit ratios are gearRatioLevel 1-3<br>
     * Gear Ratio Upgrade Kit ratios are gearRatioLevel 4-8
     */
    public static Supplier<SwerveModuleSimulation> ofMAXSwerve(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 5.5;
                    case 2 -> 5.08;
                    case 3 -> 4.71;
                    case 4 -> 4.50;
                    case 5 -> 4.29;
                    case 6 -> 4;
                    case 7 -> 3.75;
                    case 8 -> 3.56;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                9424.0 / 203.0,
                Volts.of(0.1),
                Volts.of(0.1),
                Inches.of(1.5),
                KilogramSquareMeters.of(0.02),
                wheelCOF);
    }

    /**
     * Creates a <a href="https://www.thethriftybot.com/products/thrifty-swerve">TTB Thrifty Swerve swerve module</a>
     * for simulation
     */
    public static Supplier<SwerveModuleSimulation> ofThriftySwerve(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return () -> new SwerveModuleSimulation(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 6.75;
                    case 2 -> 6.23;
                    case 3 -> 5.79;
                    case 4 -> 6;
                    case 5 -> 5.54;
                    case 6 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                25,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     *
     *
     * <h2>Creates the Simulation for a <a href="https://store.ctr-electronics.com/pigeon-2/">CTRE Pigeon 2 IMU</a>.
     * </h2>
     *
     * @return a gyro simulation factory configured for the Pigeon 2 IMU
     */
    public static Supplier<GyroSimulation> ofPigeon2() {
        /*
         * user manual of pigeon 2:
         * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
         * */
        return () -> new GyroSimulation(0.5, 0.02);
    }

    /**
     *
     *
     * <h2>Creates the Simulation for a <a href="https://pdocs.kauailabs.com/navx-mxp/">navX2-MXP IMU</a>.</h2>
     *
     * @return a gyro simulation factory configured for the navX2-MXP IMU
     */
    public static Supplier<GyroSimulation> ofNav2X() {
        return () -> new GyroSimulation(2, 0.04);
    }

    /**
     *
     *
     * <h2>Creates the Simulation for a Generic, Low-Accuracy IMU.</h2>
     *
     * @return a gyro simulation factory configured for a generic low-accuracy IMU
     */
    public static Supplier<GyroSimulation> ofGenericGyro() {
        return () -> new GyroSimulation(5, 0.06);
    }
}
