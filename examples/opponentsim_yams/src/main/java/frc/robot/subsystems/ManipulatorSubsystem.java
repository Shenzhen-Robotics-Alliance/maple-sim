package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.opponentsim.ManipulatorSim;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ManipulatorSubsystem extends SubsystemBase
{
    // TODO: Add detailed comments explaining the example, similar to the ExponentiallyProfiledArmSubsystem

    private final SparkMax                   armMotor    = new SparkMax(12, MotorType.kBrushless);
    //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
            .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController       motor       = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motor)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
            .withSoftLimit(RPM.of(-500), RPM.of(500))
            .withSpeedometerSimulation(RPM.of(750));
    private final FlyWheel       shooter       = new FlyWheel(shooterConfig);

    private final ManipulatorSim manipSim = new ManipulatorSim();

    public ManipulatorSubsystem(SwerveSubsystem swerve) {
        if (RobotBase.isSimulation()) {
            var drivetrainSim = swerve.getDrive().getConfig().getMapleDriveSim().get();
            manipSim
                    .addIntakeSimulation("Intake",
                            IntakeSimulation.InTheFrameIntake(
                                    "Coral",
                                    drivetrainSim.getDriveTrainSimulation(),
                                    Inches.of(20),
                                    IntakeSimulation.IntakeSide.FRONT,
                                    1))
                    .addProjectileSimulation("Coral",
                            () -> new ReefscapeCoralOnFly(
                                    // Get the pose of the robot so we can score from it.
                                    drivetrainSim.getActualPoseInSimulationWorld().getTranslation(),
                                    // Where the game piece should spawn relative to the robot.
                                    new Translation2d(Inches.of(0), Inches.of(-18)),
                                    /// Now we want this piece to leave the robot horizontally, so we have to do things a
                                    // little differently.
                                    // The robot speeds to help determine how fast the game piece should be when leaving the
                                    // robot.
                                    drivetrainSim
                                            .getActualSpeedsFieldRelative()
                                            // Now, to get our horizontal piece to fly correctly, we change the robot
                                            // speeds.
                                            .plus(ChassisSpeeds.fromRobotRelativeSpeeds(
                                                    // Since our scorer it
                                                    new ChassisSpeeds(
                                                            1, 0, 0), // Added chassis speeds to change coral velocity,
                                                    drivetrainSim
                                                            .getActualPoseInSimulationWorld()
                                                            .getRotation())), // this is because we want a horizontal score
                                    drivetrainSim
                                            .getActualPoseInSimulationWorld()
                                            .getRotation()
                                            .rotateBy(
                                                    Rotation2d.kCCW_90deg), // Rotated by 90 degrees for horizontal shooter.
                                    Inches.of(30), // Shooter Height
                                    MetersPerSecond.of(0), // Initial piece speed
                                    Degrees.of(0))); // Shooter angle
        }
    }

    public AngularVelocity getVelocity() {return shooter.getSpeed();}

    public Command intake()
    {
        return RobotBase.isSimulation()
                ? manipSim.intake("Intake")
                : runEnd(() -> shooter.set(-1),
                () -> shooter.set(0));
    }

    public Command score()
    {
        return RobotBase.isSimulation()
                ? manipSim.score("Coral")
                : runEnd(() -> shooter.set(1),
                () -> shooter.set(0));
    }

    public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

    public Command setDutyCycle(double dutyCycle) {return shooter.set(dutyCycle);}

    public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.setSpeed(speed);}

    public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

    public Command sysId() {return shooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}

    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }
}