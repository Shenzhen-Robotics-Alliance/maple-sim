package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

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
    public static SwerveModuleSimulationConfig ofMark4(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleSimulationConfig(
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
    public static SwerveModuleSimulationConfig ofMark4i(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleSimulationConfig(
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
    public static SwerveModuleSimulationConfig ofMark4n(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleSimulationConfig(
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
     */
    public static SwerveModuleSimulationConfig ofSwerveX(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel, double firstStageRatio) {
        double secondStageRatio =
                switch (gearRatioLevel) {
                    case 1 -> 26.0 / 20.0;
                    case 2, 3 -> 28.0 / 18.0;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                };
        return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                firstStageRatio * secondStageRatio,
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
    public static SwerveModuleSimulationConfig ofSwerveXFlipped(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel, int pinionSize) {
        var unknownPinionErr = new IllegalStateException("Unknown pinion size: " + pinionSize);
        var unknownLevelErr = new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
        return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> switch (pinionSize) {
                        case 10 -> 8.1;
                        case 11 -> 7.36;
                        case 12 -> 6.75;
                        default -> throw unknownPinionErr;
                    };
                    case 2 -> switch (pinionSize) {
                        case 10 -> 6.72;
                        case 11 -> 6.11;
                        case 12 -> 5.6;
                        default -> throw unknownPinionErr;
                    };
                    case 3 -> switch (pinionSize) {
                        case 10 -> 5.51;
                        case 11 -> 5.01;
                        case 12 -> 4.59;
                        default -> throw unknownPinionErr;
                    };
                    default -> throw unknownLevelErr;
                },
                13.3714,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-xs">WCP SwerveXS Swerve
     * Module</a> for simulation
     */
    public static SwerveModuleSimulationConfig ofSwerveXS(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel, int pinionSize) {
        var unknownPinionErr = new IllegalStateException("Unknown pinion size: " + pinionSize);
        var unknownLevelErr = new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
        return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> switch (pinionSize) {
                        case 12 -> 6;
                        case 13 -> 5.54;
                        case 14 -> 5.14;
                        default -> throw unknownPinionErr;
                    };
                    case 2 -> switch (pinionSize) {
                        case 12 -> 4.71;
                        case 13 -> 4.4;
                        case 14 -> 4.13;
                        default -> throw unknownPinionErr;
                    };
                    default -> throw unknownLevelErr;
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
    public static SwerveModuleSimulationConfig ofSwerveX2(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel, int pinionSize) {
        var unknownPinionErr = new IllegalStateException("Unknown pinion size: " + pinionSize);
        var unknownLevelErr = new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
        return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> switch (pinionSize) {
                        case 10 -> 7.67;
                        case 11 -> 6.98;
                        case 12 -> 6.39;
                        default -> throw unknownPinionErr;
                    };
                    case 2 -> switch (pinionSize) {
                        case 10 -> 6.82;
                        case 11 -> 6.2;
                        case 12 -> 5.68;
                        default -> throw unknownPinionErr;
                    };
                    case 3 -> switch (pinionSize) {
                        case 10 -> 6.48;
                        case 11 -> 5.89;
                        case 12 -> 5.4;
                        default -> throw unknownPinionErr;
                    };
                    case 4 -> switch (pinionSize) {
                        case 10 -> 5.67;
                        case 11 -> 5.15;
                        case 12 -> 4.73;
                        default -> throw unknownPinionErr;
                    };
                    default -> throw unknownLevelErr;
                },
                12.1,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    /**
     * creates a <a href="https://wcproducts.com/collections/gearboxes/products/swerve-x2-s">WCP SwerveX2S Swerve
     * Module</a> for simulation
     */
    public static SwerveModuleSimulationConfig ofSwerveX2S(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel, int pinionSize) {
        var unknownPinionErr = new IllegalStateException("Unknown pinion size: " + pinionSize);
        var unknownLevelErr = new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
        return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> switch (pinionSize) {
                        case 15 -> 6.0;
                        case 16 -> 5.63;
                        case 17 -> 5.29;
                        default -> throw unknownPinionErr;
                    };
                    case 2 -> switch (pinionSize) {
                        case 17 -> 4.94;
                        case 18 -> 4.67;
                        case 19 -> 4.42;
                        default -> throw unknownPinionErr;
                    };
                    case 3 -> switch (pinionSize) {
                        case 19 -> 4.11;
                        case 20 -> 3.9;
                        case 21 -> 3.71;
                        default -> throw unknownPinionErr;
                    };
                    default -> throw unknownLevelErr;
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
    public static SwerveModuleSimulationConfig ofMAXSwerve(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleSimulationConfig(
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
    public static SwerveModuleSimulationConfig ofThriftySwerve(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return new SwerveModuleSimulationConfig(
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
