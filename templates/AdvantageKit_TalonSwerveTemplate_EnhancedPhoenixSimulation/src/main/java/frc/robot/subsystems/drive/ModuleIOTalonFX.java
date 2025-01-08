package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public abstract class ModuleIOTalonFX implements ModuleIO {
    protected final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    protected final TalonFX driveTalon;
    protected final TalonFX turnTalon;
    protected final CANcoder cancoder;

    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    protected final StatusSignal<Angle> drivePosition;
    protected final StatusSignal<AngularVelocity> driveVelocity;
    protected final StatusSignal<Voltage> driveAppliedVolts;
    protected final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    protected final StatusSignal<Angle> turnAbsolutePosition;
    protected final StatusSignal<AngularVelocity> turnVelocity;
    protected final StatusSignal<Voltage> turnAppliedVolts;
    protected final StatusSignal<Current> turnCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    protected ModuleIOTalonFX(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        if (Constants.currentMode == Constants.Mode.SIM)
            turnConfig.Slot0.withKD(0.5).withKS(0); // during simulation, gains are slightly different

        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;};
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        // Create turn status signals
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(Drive.ODOMETRY_FREQUENCY, turnAbsolutePosition, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnVelocity, turnAppliedVolts, turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
        var turnStatus = BaseStatusSignal.refreshAll(turnVelocity, turnAppliedVolts, turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad =
                Units.rotationsToRadians(drivePosition.getValueAsDouble()) / constants.DriveMotorGearRatio;
        inputs.driveVelocityRadPerSec =
                Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / constants.DriveMotorGearRatio;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        // Update turn inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(output);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(output);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setDriveVelocity(double wheelVelocityRadPerSec) {
        double motorVelocityRotPerSec =
                Units.radiansToRotations(wheelVelocityRadPerSec) * constants.DriveMotorGearRatio;
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest.withVelocity(motorVelocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(motorVelocityRotPerSec);
                });
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.getRotations());
                });
    }
}
