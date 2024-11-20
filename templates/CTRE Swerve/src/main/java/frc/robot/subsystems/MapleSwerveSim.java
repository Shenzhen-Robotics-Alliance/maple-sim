package frc.robot.subsystems;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SimplifiedSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;

public class MapleSwerveSim {
    private final BreakerSwerveDriveSimulationBase driveSim;
    public MapleSwerveSim(CommandSwerveDrivetrain drivetrain, SwerveSimulationConfig config, SwerveModuleConstants... swerveModuleConstants) {
        SwerveModuleConstants leadModuleConstants = swerveModuleConstants[0];


        Supplier<GyroSimulation> gyroSim = GyroSimulation.getPigeon2();
        Supplier<SwerveModuleSimulation> swerveModuleSim = () -> new SwerveModuleSimulation(
            config.getDriveMotor(), 
            config.getSteerMotor(), 
            leadModuleConstants.SlipCurrent, 
            leadModuleConstants.DriveMotorGearRatio, 
            leadModuleConstants.SteerMotorGearRatio, 
            leadModuleConstants.DriveFrictionVoltage, 
            leadModuleConstants.SteerFrictionVoltage, 
            config.getTireCoefficientOfFriction(), 
            leadModuleConstants.WheelRadius, 
            leadModuleConstants.SteerInertia);

            driveSim = new BreakerSwerveDriveSimulationBase(
            new DriveTrainSimulationConfig(
                config.getRobotMass().in(Units.Kilograms), 
                config.getBumperLengthX().in(Units.Meters), 
                config.getBumperWidthY().in(Units.Meters), 
                Math.abs(leadModuleConstants.LocationX * 2), 
                Math.abs(leadModuleConstants.LocationY * 2), 
                swerveModuleSim, 
                gyroSim),
            new Pose2d(2,2,new Rotation2d()),
            drivetrain,
            swerveModuleConstants
            );
          
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
    }

    private static Angle getDeltaAngle(Angle start, Angle end) {
        double errorBound = (0.5 - (-0.5)) / 2.0;
        return Units.Rotations.of(MathUtil.inputModulus(end.in(Units.Rotations) - start.in(Units.Rotations), -errorBound, errorBound));
    }


    private static class BreakerSwerveDriveSimulationBase extends SwerveDriveSimulation {
        private BreakerSwerveModuleSim[] moduleSims;
        private Pigeon2SimState gyroSim;
        private Pigeon2 gyro;
        private CommandSwerveDrivetrain drivetrain;
        public BreakerSwerveDriveSimulationBase(DriveTrainSimulationConfig config, Pose2d initialPoseOnField, CommandSwerveDrivetrain drivetrain, SwerveModuleConstants[] moduleConstants) {
            super(config, initialPoseOnField);
            moduleSims = new BreakerSwerveModuleSim[config.moduleTranslations.length];
            SwerveModuleSimulation[] mapleModuleSims = getModules();
            SwerveModule[] modules = drivetrain.getModules();
            for (int i = 0; i < config.moduleTranslations.length; i++) {
                moduleSims[i] = new BreakerSwerveModuleSim(mapleModuleSims[i], modules[i], moduleConstants[i]);
            }
            gyro = drivetrain.getPigeon2();
            gyroSim = gyro.getSimState();
            this.drivetrain = drivetrain;
        }

        @Override
        public void simulationSubTick() {
            double supplyVoltage = RobotController.getBatteryVoltage();
            Rotation2d prevGyroAngle = gyro.getRotation2d();
            for (BreakerSwerveModuleSim moduleSim: moduleSims) {
                moduleSim.updateModel(supplyVoltage);
            }
            gyroSim.setSupplyVoltage(supplyVoltage);
            super.simulationSubTick();
            Angle gyroDelta = getDeltaAngle(Units.Radians.of(MathUtil.angleModulus(prevGyroAngle.getRadians())), getGyroSimulation().getGyroReading().getMeasure());
            for (BreakerSwerveModuleSim moduleSim: moduleSims) {
                moduleSim.updateHardware();
            }
            gyroSim.setRawYaw(prevGyroAngle.getMeasure().plus(gyroDelta));
            gyroSim.setAngularVelocityZ(Units.RadiansPerSecond.of(gyroSimulation.getMeasuredAngularVelocityRadPerSec()));
            DogLog.log("Sim/GroundTruthPose", getSimulatedDriveTrainPose());
            DogLog.log("Sim/TgtModuleStates", drivetrain.getState().ModuleTargets);

            DogLog.log("Sim/RealModuleStates", drivetrain.getState().ModuleStates);
        } 
    }

    public static class BreakerSwerveModuleSim {
        private SwerveModuleSimulation moduleSim;
        private SwerveModule module;
        private SwerveModuleConstants constants;
        private TalonFXSimState driveSim;
        private TalonFXSimState steerSim;
        private CANcoderSimState encoderSim;
        private CANcoder encoder;

        public BreakerSwerveModuleSim(SwerveModuleSimulation moduleSim, SwerveModule module, SwerveModuleConstants constants) {
            this.moduleSim = moduleSim;
            this.module = module;
            this.constants = constants;
            this.module = module;
            driveSim = module.getDriveMotor().getSimState();
            steerSim = module.getSteerMotor().getSimState();
            encoder = module.getCANcoder();
            encoderSim = encoder.getSimState();

        }
        

        public void updateModel(double supplyVoltage) {
            driveSim.Orientation = constants.DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            steerSim.Orientation = constants.SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            
            driveSim.setSupplyVoltage(supplyVoltage);
            steerSim.setSupplyVoltage(supplyVoltage);
            encoderSim.setSupplyVoltage(supplyVoltage);

            moduleSim.requestDriveVoltageOut(driveSim.getMotorVoltage());
            moduleSim.requestSteerVoltageOut(steerSim.getMotorVoltage());
        }

        public void updateHardware() {
            driveSim.setRawRotorPosition(moduleSim.getDriveEncoderUnGearedPositionRad() / (2 * Math.PI));
            driveSim.setRotorVelocity(moduleSim.getDriveEncoderUnGearedSpeedRadPerSec() / (2 * Math.PI));
            
            steerSim.setRawRotorPosition(moduleSim.getSteerRelativeEncoderPositionRad() / (2 * Math.PI));
            steerSim.setRotorVelocity(moduleSim.getSteerRelativeEncoderSpeedRadPerSec() / (2 * Math.PI));

            Angle delta = getDeltaAngle(encoder.getAbsolutePosition().getValue(), moduleSim.getSteerAbsoluteFacing().getMeasure());
            encoderSim.setRawPosition( moduleSim.getSteerAbsoluteFacing().getMeasure());
            encoderSim.setVelocity(moduleSim.getSteerAbsoluteEncoderSpeedRadPerSec() / (2 * Math.PI));
        }
    }

    public static class SwerveSimulationConfig {
        private DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);
        private DCMotor steerMotor = DCMotor.getFalcon500Foc(1);
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
