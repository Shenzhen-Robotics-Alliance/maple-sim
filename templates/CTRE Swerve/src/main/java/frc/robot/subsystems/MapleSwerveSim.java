package frc.robot.subsystems;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.ControlRequest.CustomMotorController;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

public class MapleSwerveSim {
    private final SwerveDriveSimulationBase driveSim;
    public MapleSwerveSim(CommandSwerveDrivetrain drivetrain, SwerveSimulationConfig config, SwerveModuleConstants... swerveModuleConstants) {
        SwerveModuleConstants leadModuleConstants = swerveModuleConstants[0];


        Supplier<GyroSimulation> gyroSim = GyroSimulation.getPigeon2();
        Supplier<SwerveModuleSimulation> swerveModuleSim = () -> new SwerveModuleSimulation(
            config.getDriveMotor(), 
            config.getSteerMotor(), 
            leadModuleConstants.DriveMotorGearRatio, 
            leadModuleConstants.SteerMotorGearRatio, 
            Units.Amps.of(leadModuleConstants.SlipCurrent), 
            config.getSteerMotorCurrentLimit(),
            Units.Volts.of(leadModuleConstants.DriveFrictionVoltage), 
            Units.Volts.of(leadModuleConstants.SteerFrictionVoltage), 
            Units.Meters.of(leadModuleConstants.WheelRadius), 
            Units.KilogramSquareMeters.of(leadModuleConstants.SteerInertia),
            config.getTireCoefficientOfFriction() 
            );

        Translation2d[] moduleLocations = new Translation2d[swerveModuleConstants.length];
        for (int i = 0; i < swerveModuleConstants.length; i++) {
            moduleLocations[i] = new Translation2d(swerveModuleConstants[i].LocationX, swerveModuleConstants[i].LocationY);
        }

        driveSim = new SwerveDriveSimulationBase(
        new DriveTrainSimulationConfig(
            config.getRobotMass(), 
            config.getBumperLengthX(), 
            config.getBumperWidthY(), 
            Units.Meters.of(1.0),
            Units.Meters.of(1.0),
            swerveModuleSim, 
            gyroSim)
            .withCustomModuleTranslations(moduleLocations),
        new Pose2d(2,2,new Rotation2d()),
        drivetrain,
        swerveModuleConstants
        );

        
          
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        SimulatedArena.getInstance().clearGamePieces();
    }

    private static Angle getDeltaAngle(Angle start, Angle end) {
        start = Units.Radians.of(MathUtil.angleModulus(start.in(Units.Radians)));
        end = Units.Radians.of(MathUtil.angleModulus(end.in(Units.Radians)));
        double errorBound = (0.5 - (-0.5)) / 2.0;
        return Units.Rotations.of(MathUtil.inputModulus(end.in(Units.Rotations) - start.in(Units.Rotations), -errorBound, errorBound));
    }


    private static class SwerveDriveSimulationBase extends SwerveDriveSimulation {
        private SwerveModuleSim[] moduleSims;
        private Pigeon2SimState gyroSim;
        private Pigeon2 gyro;
        private CommandSwerveDrivetrain drivetrain;
        private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
        public SwerveDriveSimulationBase(DriveTrainSimulationConfig config, Pose2d initialPoseOnField, CommandSwerveDrivetrain drivetrain, SwerveModuleConstants[] moduleConstants) {
            super(config, initialPoseOnField);
            moduleSims = new SwerveModuleSim[config.moduleTranslations.length];
            SwerveModuleSimulation[] mapleModuleSims = getModules();
            SwerveModule[] modules = drivetrain.getModules();
            for (int i = 0; i < config.moduleTranslations.length; i++) {
                moduleSims[i] = new SwerveModuleSim(mapleModuleSims[i], modules[i], moduleConstants[i]);
            }
            gyro = drivetrain.getPigeon2();
            gyroSim = gyro.getSimState();
            this.drivetrain = drivetrain;
        }

        @Override
        public void simulationSubTick() {
            double supplyVoltage = RobotController.getBatteryVoltage();
            Angle prevGyroAngle = gyro.getYaw().getValue();
            gyroSim.setSupplyVoltage(supplyVoltage);
            super.simulationSubTick();
            Angle gyroDelta = getDeltaAngle(prevGyroAngle, getGyroSimulation().getGyroReading().getMeasure());
            gyroSim.setRawYaw(prevGyroAngle.plus(gyroDelta));
            gyroSim.setAngularVelocityZ(gyroSimulation.getMeasuredAngularVelocity());
            DogLog.log("Sim/GroundTruthPose", getSimulatedDriveTrainPose());
            DogLog.log("Sim/TgtModuleStates", drivetrain.getState().ModuleTargets);
            DogLog.log("Sim/RealModuleStates", drivetrain.getState().ModuleStates);
            DogLog.log("Sim/GyroYaw", MathUtil.inputModulus(prevGyroAngle.in(Units.Degrees), -180, 180));
            DogLog.log("Sim/GroundTruthAccels", getDriveTrainSimulatedChassisSpeedsFieldRelative().minus(prevSpeeds).div(SimulatedArena.getSimulationDt().in(Units.Seconds)));
            prevSpeeds = getDriveTrainSimulatedChassisSpeedsFieldRelative();
            prevSpeeds = new ChassisSpeeds(prevSpeeds.vxMetersPerSecond, prevSpeeds.vyMetersPerSecond, prevSpeeds.omegaRadiansPerSecond);
        } 
    }

    private static class SwerveModuleSim {
        public SwerveModuleSim(SwerveModuleSimulation moduleSim, SwerveModule module, SwerveModuleConstants constants) {
            DriveMotorSim driveSim = new DriveMotorSim(module.getDriveMotor().getSimState(), constants.DriveMotorInverted);
            SteerMotorSim steerSim = new SteerMotorSim(module.getSteerMotor().getSimState(), module.getCANcoder().getSimState(), constants.SteerMotorInverted, constants.CANcoderInverted);
            moduleSim.requestDriveControl(driveSim);
            moduleSim.requestSteerControl(steerSim);
        }

        private static class SteerMotorSim implements CustomMotorController {
            private TalonFXSimState motorSim;
            private CANcoderSimState encoderSim;
            private boolean isMotorInverted, isEncoderInverted;

            public SteerMotorSim(TalonFXSimState motorSim, CANcoderSimState encoderSim, boolean isMotorInverted, boolean isEncoderInverted) {
                this.motorSim = motorSim;
                this.encoderSim = encoderSim;
                this.isMotorInverted = isMotorInverted;
                this.isEncoderInverted = isEncoderInverted;
            }

            @Override
            public void feedEncoderPosition(Angle encoderPosition) {
                motorSim.setRawRotorPosition(encoderPosition);
            }

            @Override
            public void feedEncoderVelocity(AngularVelocity encoderVelocity) {
                motorSim.setRotorVelocity(encoderVelocity);
            }

            @Override
            public void feedMechanismPosition(Angle mechanismPosition) {
                encoderSim.setRawPosition(mechanismPosition);
            }

            @Override
            public void feedMechanismVelocity(AngularVelocity mechanismVelocity) {
                encoderSim.setVelocity(mechanismVelocity);
            }

            @Override
            public void updateSimulation(Time dt) {
            }

            @Override
            public Voltage getMotorVoltage() {
                return motorSim.getMotorVoltageMeasure();
            }

            @Override
            public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
                motorSim.Orientation = isMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
                encoderSim.Orientation = isEncoderInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
                double supplyVoltage = RobotController.getBatteryVoltage();
                motorSim.setSupplyVoltage(supplyVoltage);
                encoderSim.setSupplyVoltage(supplyVoltage);
                return CustomMotorController.super.updateSignal(configs, mechanismAngle, mechanismVelocity);
            }

        }

        private static class DriveMotorSim implements CustomMotorController {
            private TalonFXSimState motorSim;
            private boolean isMotorInverted;

            public DriveMotorSim(TalonFXSimState motorSim, boolean isMotorInverted) {
                this.motorSim = motorSim;
                this.isMotorInverted = isMotorInverted;
            }

            @Override
            public void feedEncoderPosition(Angle encoderPosition) {
                motorSim.setRawRotorPosition(encoderPosition);
            }

            @Override
            public void feedEncoderVelocity(AngularVelocity encoderVelocity) {
                motorSim.setRotorVelocity(encoderVelocity);
            }

            @Override
            public void feedMechanismPosition(Angle mechanismPosition) {
            }

            @Override
            public void feedMechanismVelocity(AngularVelocity mechanismVelocity) {
            }

            @Override
            public void updateSimulation(Time dt) {
            }

            @Override
            public Voltage getMotorVoltage() {
                return motorSim.getMotorVoltageMeasure();
            }

            @Override
            public Voltage updateSignal(SimMotorConfigs configs, Angle mechanismAngle, AngularVelocity mechanismVelocity) {
                motorSim.Orientation = isMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
                double supplyVoltage = RobotController.getBatteryVoltage();
                motorSim.setSupplyVoltage(supplyVoltage);
                return CustomMotorController.super.updateSignal(configs, mechanismAngle, mechanismVelocity);
            }

        }
    }

    public static class SwerveSimulationConfig {
        private DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);
        private DCMotor steerMotor = DCMotor.getFalcon500Foc(1);
        private Current steerMotorCurrentLimit = Units.Amps.of(60);
        private double tireCoefficientOfFriction = 1.01;
        private Mass robotMass = Units.Pound.of(110);
        private Distance bumperLengthX = Units.Inches.of(35);
        private Distance bumperWidthY = Units.Inches.of(35);

        public DCMotor getDriveMotor() {
            return driveMotor;
        }

        public DCMotor getSteerMotor() {
            return steerMotor;
        }

        public Current getSteerMotorCurrentLimit() {
            return steerMotorCurrentLimit;
        }

        public double getTireCoefficientOfFriction() {
            return tireCoefficientOfFriction;
        }

        public Mass getRobotMass() {
            return robotMass;
        }

        public Distance getBumperLengthX() {
            return bumperLengthX;
        }

        public Distance getBumperWidthY() {
            return bumperWidthY;
        }
    }
}
