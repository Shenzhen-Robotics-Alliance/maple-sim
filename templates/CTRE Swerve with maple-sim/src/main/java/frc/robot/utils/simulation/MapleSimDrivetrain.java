package frc.robot.utils.simulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

public class MapleSimDrivetrain {
    private final double periodSeconds;
    private final SwerveDriveSimulation driveSimulation;
    private final MapleSimModule[] simModules;
    private final Pigeon2SimState imuSim;

    public MapleSimDrivetrain(
            double simPeriodSeconds,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            Pigeon2 imu
    ) {
        this.simModules = new MapleSimModule[moduleLocations.length];
        DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(robotMassWithBumpers)
                .withBumperSize(bumperLengthX, bumperWidthY)
                .withGyro(COTS.ofPigeon2())
                .withCustomModuleTranslations(moduleLocations)
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        driveMotorModel,
                        steerMotorModel,
                        moduleConstants[0].DriveMotorGearRatio,
                        moduleConstants[1].SteerMotorGearRatio,
                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                        Meters.of(moduleConstants[0].WheelRadius),
                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                        wheelCOF));
        driveSimulation = new SwerveDriveSimulation(simulationConfig, new Pose2d());

        for (int i = 0; i < simModules.length; i++)
            simModules[i] = new MapleSimModule(
                    driveSimulation.getModules()[i],
                    moduleConstants[i].DriveMotorInverted,
                    moduleConstants[i].SteerMotorInverted,
                    moduleConstants[i].EncoderInverted,
                    modules[i]);
        this.imuSim = imu.getSimState();

        this.periodSeconds = simPeriodSeconds;
        SimulatedArena.overrideSimulationTimings(Seconds.of(periodSeconds), 1);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    }

    public SwerveDriveSimulation driveSimulation() {
        return driveSimulation;
    }

    public void update() {
       SimulatedArena.getInstance().simulationPeriodic();
       imuSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
       imuSim.setRawYaw(driveSimulation.getSimulatedDriveTrainPose().getRotation().getMeasure());
       imuSim.setAngularVelocityZ(RadiansPerSecond.of(
               driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
    }

    protected static class MapleSimModule {
        public final SwerveModuleSimulation moduleSimulation;
        protected MapleSimModule(
                SwerveModuleSimulation moduleSimulation,
                boolean driveMotorInverted, boolean steerMotorInverted, boolean encoderInverted,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(
                    module.getDriveMotor(), driveMotorInverted));
            moduleSimulation.useSteerMotorController(new TalonFXMotorControllerWithRemoteCanCoderSim(
                    module.getSteerMotor(), steerMotorInverted, module.getEncoder(), encoderInverted, Degrees.zero()));
        }
    }

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState talonFXSimState;
        private final boolean motorInverted;
        public TalonFXMotorControllerSim(TalonFX talonFX, boolean motorInverted) {
            this.id = talonFX.getDeviceID();
            this.motorInverted = motorInverted;
            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.Orientation =
                    motorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            DogLog.log("CTRE Sim/Motor" + id + "/Inverted", motorInverted);
            DogLog.log("CTRE Sim/Motor" + id + "/EncoderAngle (Rot)", encoderAngle.in(Rotations));
            DogLog.log("CTRE Sim/Motor" + id + "/EncoderVelocity (RPM)", encoderVelocity.in(RPM));
            DogLog.log("CTRE Sim/Motor" + id + "/Output Voltage", talonFXSimState.getMotorVoltageMeasure().in(Volts));
            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
        private final int encoderId;
        private final boolean encoderInverted;
        private final CANcoderSimState remoteCancoderSimState;
        private final Angle encoderOffset;
        public TalonFXMotorControllerWithRemoteCanCoderSim(
                    TalonFX talonFX,
                    boolean motorInverted,
                    CANcoder cancoder,
                    boolean encoderInverted,
                    Angle encoderOffset) {
                super(talonFX, motorInverted);
                this.remoteCancoderSimState = cancoder.getSimState();
                this.encoderInverted = encoderInverted;
                this.encoderOffset = encoderOffset;

                this.encoderId = cancoder.getDeviceID();
            }

            @Override
            public Voltage updateControlSignal(
                    Angle mechanismAngle,
                    AngularVelocity mechanismVelocity,
                    Angle encoderAngle,
                    AngularVelocity encoderVelocity) {
                this.remoteCancoderSimState.Orientation =
                        encoderInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

                remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
                remoteCancoderSimState.setRawPosition(mechanismAngle.minus(encoderOffset));
                remoteCancoderSimState.setVelocity(mechanismVelocity);

                System.out.println("encoder id " + encoderId +" inverted: " + encoderInverted);
                DogLog.log("CTRE Sim/CanCoder" + encoderId + "/Inverted", encoderInverted);
                DogLog.log("CTRE Sim/CanCoder" + encoderId + "/EncoderAngle (Rot)", mechanismAngle.in(Rotations));
                DogLog.log("CTRE Sim/CanCoder" + encoderId + "/EncoderVelocity (RPM)", mechanismVelocity.in(RPM));

                return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
            }
    }
}
