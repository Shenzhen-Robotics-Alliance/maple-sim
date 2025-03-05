package frc.robot.utils.simulation;

// Copyright 2021-2025 Iron Maple 5516
// Original Source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/CTRE%20Swerve%20with%20maple-sim/src/main/java/frc/robot/utils/simulation/MapleSimSwerveDrivetrain.java
//
// This code is licensed under MIT license (see https://mit-license.org/)

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 *
 *
 * <h2>Injects Maple-Sim simulation data into a CTRE swerve drivetrain.</h2>
 *
 * <p>This class retrieves simulation data from Maple-Sim and injects it into the CTRE
 * {@link com.ctre.phoenix6.swerve.SwerveDrivetrain} instance.
 *
 * <p>It replaces the {@link com.ctre.phoenix6.swerve.SimSwerveDrivetrain} class.
 */
public class MapleSimSwerveDrivetrain {
    private final Pigeon2SimState pigeonSim;
    private final SimSwerveModule[] simModules;
    public final SwerveDriveSimulation mapleSimDrive;

    /**
     *
     *
     * <h2>Constructs a drivetrain simulation using the specified parameters.</h2>
     *
     * @param simPeriod the time period of the simulation
     * @param robotMassWithBumpers the total mass of the robot, including bumpers
     * @param bumperLengthX the length of the bumper along the X-axis (influences the collision space of the robot)
     * @param bumperWidthY the width of the bumper along the Y-axis (influences the collision space of the robot)
     * @param driveMotorModel the {@link DCMotor} model for the drive motor, typically <code>DCMotor.getKrakenX60Foc()
     *     </code>
     * @param steerMotorModel the {@link DCMotor} model for the steer motor, typically <code>DCMotor.getKrakenX60Foc()
     *     </code>
     * @param wheelCOF the coefficient of friction of the drive wheels
     * @param moduleLocations the locations of the swerve modules on the robot, in the order <code>FL, FR, BL, BR</code>
     * @param pigeon the {@link Pigeon2} IMU used in the drivetrain
     * @param modules the {@link SwerveModule}s, typically obtained via {@link SwerveDrivetrain#getModules()}
     * @param moduleConstants the constants for the swerve modules
     */
    public MapleSimSwerveDrivetrain(
            Time simPeriod,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            Pigeon2 pigeon,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
                    moduleConstants) {
        this.pigeonSim = pigeon.getSimState();
        simModules = new SimSwerveModule[moduleConstants.length];
        DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(robotMassWithBumpers)
                .withBumperSize(bumperLengthX, bumperWidthY)
                .withGyro(COTS.ofPigeon2())
                .withCustomModuleTranslations(moduleLocations)
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        driveMotorModel,
                        steerMotorModel,
                        moduleConstants[0].DriveMotorGearRatio,
                        moduleConstants[0].SteerMotorGearRatio,
                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                        Meters.of(moduleConstants[0].WheelRadius),
                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                        wheelCOF));
        mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

        SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
        for (int i = 0; i < this.simModules.length; i++)
            simModules[i] = new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);

        SimulatedArena.overrideSimulationTimings(simPeriod, 1);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    /**
     *
     *
     * <h2>Update the simulation.</h2>
     *
     * <p>Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices, including motors and
     * the IMU.
     */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        pigeonSim.setRawYaw(
                mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
                mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
    }

    /**
     *
     *
     * <h1>Represents the simulation of a single {@link SwerveModule}.</h1>
     */
    protected static class SimSwerveModule {
        public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                moduleConstant;
        public final SwerveModuleSimulation moduleSimulation;

        public SimSwerveModule(
                SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstant,
                SwerveModuleSimulation moduleSimulation,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleConstant = moduleConstant;
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(module.getDriveMotor()));
            moduleSimulation.useSteerMotorController(
                    new TalonFXMotorControllerWithRemoteCanCoderSim(module.getSteerMotor(), module.getEncoder()));
        }
    }

    // Static utils classes
    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
        private final int encoderId;
        private final CANcoderSimState remoteCancoderSimState;

        public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();

            this.encoderId = cancoder.getDeviceID();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);

            return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    /**
     *
     *
     * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
     *
     * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary adjustments for simulation
     * purposes, ensuring compatibility and avoiding known bugs.
     *
     * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
     */
    public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
        for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants)
            regulateModuleConstantForSimulation(moduleConstant);

        return moduleConstants;
    }

    /**
     *
     *
     * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
     *
     * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation purposes. These
     * changes have no effect on real robot operations and address known simulation bugs:
     *
     * <ul>
     *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted configurations.
     *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization issues.
     *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve simulation
     *       performance.
     * </ul>
     *
     * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants used on real
     * robot hardware.</h4>
     */
    private static void regulateModuleConstantForSimulation(SwerveModuleConstants<?, ?, ?> moduleConstants) {
        // Skip regulation if running on a real robot
        if (RobotBase.isReal()) return;

        // Apply simulation-specific adjustments to module constants
        moduleConstants
                // Disable encoder offsets
                .withEncoderOffset(0)
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // Adjust steer motor PID gains for simulation
                .withSteerMotorGains(moduleConstants
                        .SteerMotorGains
                        .withKP(70) // Proportional gain
                        .withKD(4.5)
                        .withKS(0.0)) // Derivative gain
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0))
                .withSteerFrictionVoltage(Volts.of(0))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
