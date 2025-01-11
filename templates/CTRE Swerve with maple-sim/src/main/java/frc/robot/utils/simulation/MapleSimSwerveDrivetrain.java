package frc.robot.utils.simulation;

/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

public class MapleSimSwerveDrivetrain {
    protected static class SimSwerveModule {
        public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstant;
        public final SwerveModuleSimulation moduleSimulation;
        public final SimulatedMotorController.GenericMotorController driveMotor, steerMotor;
        public SimSwerveModule(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstant, SwerveModuleSimulation moduleSimulation) {
            this.moduleConstant = moduleConstant;
            this.moduleSimulation = moduleSimulation;
            this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive();
            this.steerMotor = moduleSimulation.useGenericControllerForSteer();
        }
    }

    private final Pigeon2SimState pigeonSim;
    private final SimSwerveModule[] simModules;
    public final SwerveDriveSimulation mapleSimDrive;

    public MapleSimSwerveDrivetrain(
            double simPeriodSeconds,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            Pigeon2SimState pigeonSim,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.pigeonSim = pigeonSim;
        simModules = new SimSwerveModule[4];
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
            simModules[i] = new SimSwerveModule(moduleConstants[0], moduleSimulations[i]);

        SimulatedArena.overrideSimulationTimings(Seconds.of(simPeriodSeconds), 1);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    /**
     * Update this simulation for the time duration.
     * <p>
     * This performs a simulation update on all the simulated devices
     *
     * @param modulesToApply What modules to apply the update to
     */
    public final void update(SwerveModule<TalonFX, TalonFX, CANcoder>... modulesToApply) {
        if (modulesToApply.length != simModules.length)
            throw new IllegalArgumentException("Modules length incorrect!!! (given " + modulesToApply.length +" modules)");

        for (int i = 0; i < simModules.length; ++i) {
            TalonFXSimState driveTalonSim = modulesToApply[i].getDriveMotor().getSimState();
            TalonFXSimState steerTalonSim = modulesToApply[i].getSteerMotor().getSimState();
            CANcoderSimState encoderSim = modulesToApply[i].getEncoder().getSimState();

            driveTalonSim.Orientation = simModules[i].moduleConstant.DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            steerTalonSim.Orientation = simModules[i].moduleConstant.SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            encoderSim.Orientation = simModules[i].moduleConstant.EncoderInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

            Voltage batteryVoltage = SimulatedBattery.getBatteryVoltage();
            driveTalonSim.setSupplyVoltage(batteryVoltage);
            steerTalonSim.setSupplyVoltage(batteryVoltage);
            encoderSim.setSupplyVoltage(batteryVoltage);

            simModules[i].driveMotor.requestVoltage(Volts.of(driveTalonSim.getMotorVoltage()));
            simModules[i].steerMotor.requestVoltage(Volts.of(steerTalonSim.getMotorVoltage()));

            driveTalonSim.setRawRotorPosition(simModules[i].moduleSimulation.getDriveEncoderUnGearedPosition());
            driveTalonSim.setRotorVelocity(simModules[i].moduleSimulation.getDriveEncoderUnGearedSpeed());

            steerTalonSim.setRawRotorPosition(simModules[i].moduleSimulation.getSteerRelativeEncoderPosition());
            steerTalonSim.setRotorVelocity(simModules[i].moduleSimulation.getSteerRelativeEncoderVelocity());

            encoderSim.setRawPosition(
                    simModules[i].moduleSimulation.getSteerRelativeEncoderPosition()
                            .div(simModules[i].moduleSimulation.config.STEER_GEAR_RATIO));
            encoderSim.setVelocity(simModules[i].moduleSimulation.getSteerAbsoluteEncoderSpeed());
        }
        pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
                mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));

        SimulatedArena.getInstance().simulationPeriodic();
    }
}
