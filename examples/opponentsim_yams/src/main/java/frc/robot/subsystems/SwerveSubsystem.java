package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

/**
 * Swerve Drive Subsystem, using MapleSim for physics simulation.
 * This example is taken from YASS's YAMS Library.
 */
public class SwerveSubsystem extends SubsystemBase
{
  /// Telemetry Options
  private final String                                          driveMotorTelemetryName                 = "driveMotor";
  private final String                                          angleMotorTelemetryName                 = "angleMotor";
  private final SmartMotorControllerConfig.TelemetryVerbosity   telemetryVerbosity                      = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;
  /// Module Configuration Options
  private final DCMotor                                         driveMotor                              = DCMotor.getNEO(1);
  private final DCMotor                                         azimuthMotor                            = DCMotor.getNEO(1);
  private final MechanismGearing                                driveGearing                            = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
  private final Distance                                        wheelDiameter                           = Inches.of(4);
  private final MechanismGearing                                azimuthGearing                          = new MechanismGearing(GearBox.fromStages("21:1"));
  private final PIDController                                   drivePIDController                      = new PIDController(50, 0, 4);
  private final PIDController                                   azimuthPIDController                    = new PIDController(50, 0, 4);
  private final Current                                         driveStatorCurrentLimit                 = Amps.of(40);
  private final Current                                         azimuthStatorCurrentLimit               = Amps.of(20);
  /// Module Locations
  // Track Width and height can be used to quickly set locations on symmetric robots. If overriding, make sure to update the MapleSim configuration as it uses these values.
  private final Distance                                        trackHeight                             = Inches.of(30);
  private final Distance                                        trackWidth                              = Inches.of(30);
  private final Translation2d                                   flLocation                              = new Translation2d(trackHeight.div(2), trackWidth.div(2));
  private final Translation2d                                   frLocation                              = new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus());
  private final Translation2d                                   blLocation                              = new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2));
  private final Translation2d                                   brLocation                              = new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2).unaryMinus());
  /// Drivetrain Configuration Options
  private final LinearVelocity                                  maximumChassisSpeedsLinearVelocity      = MetersPerSecond.of(10);
  private final AngularVelocity                                 maximumChassisSpeedsAngularVelocity     = DegreesPerSecond.of(360);
  // Swerve Drive PID Controllers, used in DriveToPose.
  private final PIDController                                   translationController                   = new PIDController(1, 0, 0);
  private final PIDController                                   rotationController                      = new PIDController(1, 0, 0);

  /// MapleSim Integration Options
  private final Distance                                        wheelCenterToBumper                     = Inches.of(5);
  private final Mass                                            robotMass                               = Pounds.of(100);
  private final Pose2d                                          startingPose                            = new Pose2d(1.5, 3, Rotation2d.fromDegrees(0));

  /// MapleSim GamePiece publishing
  private final StructArrayPublisher<Pose3d>                    coralPoses                              = NetworkTableInstance.getDefault()
                                                                                                            .getTable("SmartDashboard/MapleSim/GamePieces")
                                                                                                            .getStructArrayTopic("Coral Array", Pose3d.struct).publish();
  private final StructArrayPublisher<Pose3d>                    algaePoses                              = NetworkTableInstance.getDefault()
                                                                                                            .getTable("SmartDashboard/MapleSim/GamePieces")
                                                                                                            .getStructArrayTopic("Algae Array", Pose3d.struct).publish();

  /// SwerveSubsystem Global Variables
  private final SwerveDrive     drive;
  private final Field2d         field = new Field2d();

    /**
     * A helper method to easily create identical {@link SwerveModule} objects.
     * @param drive {@link SparkMax} for the drive motor.
     * @param azimuth {@link SparkMax} for the azimuth motor.
     * @param absoluteEncoder {@link CANcoder} for the absolute encoder. {@code absoluteEncoder.getAbsolutePosition().asSupplier()}
     * @param moduleName {@link String} for the module name, used for telemetry. {@code "moduleName"}
     * @param location {@link Translation2d} for the module location, also used in MapleSim simulation.
     * @return
     */
  private SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName, Translation2d location)
  {
    /// Module Configuration
    // Drive Motor Configuration
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(wheelDiameter)
        .withClosedLoopController(drivePIDController)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(driveStatorCurrentLimit)
        .withTelemetry(driveMotorTelemetryName, telemetryVerbosity);
    // Azimuth Motor Configuration
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(azimuthPIDController)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(azimuthStatorCurrentLimit)
        .withTelemetry(angleMotorTelemetryName, telemetryVerbosity);
    // Drive Motor SMC
    SmartMotorController driveSMC   = new SparkWrapper(drive, driveMotor, driveCfg);
    // Azimuth Motor SMC
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, azimuthMotor, azimuthCfg);
    // Swerve Module Configuration
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, telemetryVerbosity)
        .withLocation(location)
        .withOptimization(true);
    /// Swerve Module Object
    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem()
  {
    /// Pigeon 2 Gyro
    Pigeon2 gyro = new Pigeon2(14);
    /// Swerve Modules Construction
    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                          new SparkMax(2, MotorType.kBrushless),
                          new CANcoder(3),
                          "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                          new SparkMax(5, MotorType.kBrushless),
                          new CANcoder(6),
                          "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                          new SparkMax(8, MotorType.kBrushless),
                          new CANcoder(9),
                          "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
                          new SparkMax(11, MotorType.kBrushless),
                          new CANcoder(12),
                          "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));
    /// MapleSim drive train simulation configuration.
    final DriveTrainSimulationConfig mapleConfig = DriveTrainSimulationConfig.Default()
            .withBumperSize(trackHeight.plus(wheelCenterToBumper), trackWidth.plus(wheelCenterToBumper))
            .withRobotMass(robotMass)
            .withCustomModuleTranslations(new Translation2d[] { flLocation, frLocation, blLocation, brLocation })
            /// Module locations can also be called back from the SwerveModuleConfigs.
//            .withCustomModuleTranslations(new Translation2d[] {
//                    fl.getConfig().getLocation(), fr.getConfig().getLocation(),
//                    bl.getConfig().getLocation(), br.getConfig().getLocation()})
            .withGyro(COTS.ofPigeon2());
    /// SwerveDrive Specific Configuration
    final SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withTranslationController(translationController)
        .withRotationController(rotationController)
        .withMapleSim(mapleConfig, startingPose);
    /// SwerveDrive Mechanism
    drive = new SwerveDrive(config);
    /// Add our Field2d to the dashboard.
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic()
  {
    /// Update the telemetry of the drive.
    drive.updateTelemetry();
    // If we're running in simulation, use the simulated pose, otherwise use the odometry pose.
    field.setRobotPose(RobotBase.isSimulation() ? drive.getMapleSimPose() : drive.getPose());

  }

  @Override
  public void simulationPeriodic()
  {
    /// MapleSim must be updated every simulation loop, it doesn't matter where so long as it's called.
    SimulatedArena.getInstance().simulationPeriodic();
    /// Run the simulation loop, maplesim runs together with default YAMS simulation.
    drive.simIterate();
    // If we're running in simulation, add the odometry pose to the field.
    field.getObject("odometry").setPose(drive.getPose());
    // Get the positions of the notes (both on the field and in the air);
    coralPoses.set(SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral"));
    algaePoses.set(SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Algae"));
  }

  /**
   * Get the {@link SwerveDrive} object.
   *
   * @return {@link SwerveDrive} object.
   */
  public SwerveDrive getDrive() { return drive; }

  /**
   * Drive the {@link SwerveDrive} object with robot relative chassis speeds.
   *
   * @param speedsSupplier Robot relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
      return drive.drive(speedsSupplier);
  }

  /**
   * Set robot relative chassis speeds.
   *
   * @param speeds Robot relative chassis speeds.
   */
  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
      return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  /**
   * Drive the robot to the given pose.
   *
   * @param pose {@link Pose2d} to drive the robot to. Field relative, blue-origin where 0deg is facing towards RED
   * @return {@link Command} to drive the robot to the given pose.
   */
  public Command driveToPose(Pose2d pose)
  {
      return drive.driveToPose(pose);
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public Command lock()
  {
      return run(drive::lockPose);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 (red alliance
   * station).
   */
  public Command zeroGyro()
  {
      return runOnce(drive::zeroGyro);
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param pose The pose to set the odometry to. Field relative, blue-origin where 0deg is facing towards RED
   *             alliance.
   */
  public Command resetOdometry(Pose2d pose2d)
  {
      return runOnce(() -> drive.resetOdometry(pose2d));
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,
                                                    DoubleSupplier translationYScalar,
                                                    DoubleSupplier rotationScalar)
  {
      return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
              .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
              .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
              .withDeadband(0.01)
              .withCubeRotationControllerAxis()
              .withCubeTranslationControllerAxis()
              .withAllianceRelativeControl();
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar,
                                                        DoubleSupplier translationYScalar,
                                                        DoubleSupplier rotationScalar)
  {
      return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
              .in(MetersPerSecond),
              maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                      .in(MetersPerSecond),
              maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                      .in(RadiansPerSecond));
  }
}

