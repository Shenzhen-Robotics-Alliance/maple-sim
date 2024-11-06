package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 *
 *
 * <h1>Simulates a Swerve Drivetrain.</h1>
 *
 * <h3>Purpose</h3>
 *
 * <p>This class simulates a swerve drivetrain composed of more than two {@link
 * SwerveModuleSimulation} modules.
 *
 * <p>It provides a realistic modeling of drivetrain physics, replicating wheel grip and motor
 * propulsion for an actual swerve drive.
 *
 * <h3>Simulation Dynamics</h3>
 *
 * <ul>
 *   <li>1. Propelling forces generated by the drive motors, computed by {@link
 *       SwerveModuleSimulation#updateSimulationSubTickGetModuleForce(Vector2, Rotation2d, double)}.
 *   <li>2. Friction forces generated by the wheels that "pull" the robot from its current ground
 *       velocity to the module velocities, both translational and rotational.
 *   <li>3. Centripetal forces generated by the steering when the drivetrain makes a turn.
 * </ul>
 *
 * <h3>Odometry Simulation</h3>
 *
 * <p>To simulate odometry, follow these steps:
 *
 * <ul>
 *   <li>Obtain the {@link SwerveModuleSimulation} instances through {@link #getModules()}.
 *   <li>Create an <a
 *       href='https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md'>IO
 *       Implementation</a> that wraps around {@link SwerveModuleSimulation} to retrieve encoder
 *       readings.
 *   <li>Update a {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} using the encoder
 *       readings, similar to how you would on a real robot.
 * </ul>
 *
 * <p>Refer to the <a
 * href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java'>ModuleIOSim.java</a>
 * example project for more details.
 *
 * <h3>Vision Simulation</h3>
 *
 * <p>You can obtain the real robot pose from {@link #getSimulatedDriveTrainPose()} and feed it to
 * the <a
 * href="https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html#updating-the-simulation-world">PhotonVision
 * simulation</a> to simulate vision.
 */
public class SwerveDriveSimulation extends AbstractDriveTrainSimulation {
  private final SwerveModuleSimulation[] moduleSimulations;
  protected final GyroSimulation gyroSimulation;
  protected final Translation2d[] moduleTranslations;
  protected final SwerveDriveKinematics kinematics;
  private final Force gravityForceOnEachModule;

  /**
   *
   *
   * <h2>Creates a Swerve Drive Simulation.</h2>
   *
   * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper
   * dimensions, module simulations, module translations, gyro simulation, and initial pose on the
   * field.
   *
   * @param robotMassWithBumpersKg the total mass of the robot including bumpers, in kilograms
   * @param bumperWidthMeters the width of the robot's bumpers, in meters
   * @param bumperLengthMeters the length of the robot's bumpers, in meters
   * @param moduleSimulations an array of {@link SwerveModuleSimulation} objects representing the
   *     modules of the drivetrain
   * @param moduleTranslations an array of {@link Translation2d} objects representing the position
   *     of each module relative to the robot center
   * @param gyroSimulation a {@link GyroSimulation} object used to simulate the IMU for odometry
   * @param initialPoseOnField the initial pose of the drivetrain in the simulation world,
   *     represented as a {@link Pose2d}
   */
  public SwerveDriveSimulation(
      Mass robotMassWithBumpersKg,
      Distance bumperWidthMeters,
      Distance bumperLengthMeters,
      SwerveModuleSimulation[] moduleSimulations,
      Translation2d[] moduleTranslations,
      GyroSimulation gyroSimulation,
      Pose2d initialPoseOnField) {
    super(
        new DriveTrainSimulationProfile(
                moduleSimulations[0].getModuleTheoreticalSpeedMPS(),
                moduleSimulations[0].getModuleMaxAccelerationMPSsq(
                    robotMassWithBumpersKg, moduleSimulations.length),
                RadiansPerSecond.of(moduleSimulations[0].getModuleTheoreticalSpeedMPS()
                    .divide(moduleTranslations[0].getNorm()).in(MetersPerSecond)),
                RadiansPerSecondPerSecond.of(moduleSimulations[0].getModuleMaxAccelerationMPSsq(
                        robotMassWithBumpersKg, moduleSimulations.length)
                    .divide(moduleTranslations[0].getNorm()).in(MetersPerSecondPerSecond)),
                robotMassWithBumpersKg,
                bumperWidthMeters,
                bumperLengthMeters)
            .withAngularVelocityDamping(1.4)
            .withLinearVelocityDamping(1.4),
        initialPoseOnField);

    this.moduleSimulations = moduleSimulations;
    this.moduleTranslations = moduleTranslations;
    this.kinematics = new SwerveDriveKinematics(moduleTranslations);
    this.gyroSimulation = gyroSimulation;

    this.gravityForceOnEachModule = Newtons.of(profile.robotMass.in(Kilograms) * 9.8 / moduleSimulations.length);
  }

  /**
   *
   *
   * <h2>Constructs a Swerve Drive Simulation.</h2>
   *
   * <p>This constructor initializes a swerve drive simulation with the given robot mass, track
   * dimensions, bumper dimensions, module factory, gyro simulation, and initial pose on the field.
   * It calls the main constructor internally.
   *
   * @param robotMassWidthBumpersKg the total mass of the robot including bumpers, in kilograms
   * @param trackWidthMeters the width of the robot's track, in meters
   * @param trackLengthMeters the length of the robot's track, in meters
   * @param bumperWidthMeters the width of the robot's bumpers, in meters
   * @param bumperLengthMeters the length of the robot's bumpers, in meters
   * @param swerveModuleFactory a supplier that provides instances of {@link SwerveModuleSimulation}
   *     for each swerve module
   * @param gyroSimulation the {@link GyroSimulation} instance to simulate the robot's IMU
   * @param initialPoseOnField the initial pose of the robot in the simulation world, represented as
   *     a {@link Pose2d}
   */
  public SwerveDriveSimulation(
      Mass robotMassWidthBumpersKg,
      Distance trackWidthMeters,
      Distance trackLengthMeters,
      Distance bumperWidthMeters,
      Distance bumperLengthMeters,
      Supplier<SwerveModuleSimulation> swerveModuleFactory,
      GyroSimulation gyroSimulation,
      Pose2d initialPoseOnField) {

    // Use this() to call the main constructor with calculated parameters
    this(
        robotMassWidthBumpersKg,
        bumperWidthMeters,
        bumperLengthMeters,
        new SwerveModuleSimulation[] {
          swerveModuleFactory.get(), swerveModuleFactory.get(),
          swerveModuleFactory.get(), swerveModuleFactory.get()
        },
        new Translation2d[] {
          new Translation2d(trackLengthMeters.divide(2), trackWidthMeters.divide(2)),
          new Translation2d(trackLengthMeters.divide(2), trackWidthMeters.divide(-2)),
          new Translation2d(trackLengthMeters.divide(-2), trackWidthMeters.divide(2)),
          new Translation2d(trackLengthMeters.divide(-2), trackWidthMeters.divide(-2))
        },
        gyroSimulation,
        initialPoseOnField);
  }

  /**
   *
   *
   * <h2>Updates the Swerve Drive Simulation.</h2>
   *
   * <p>This method performs the following actions during each sub-tick of the simulation:
   *
   * <ul>
   *   <li>Applies the translational friction force to the physics engine.
   *   <li>Applies the rotational friction torque to the physics engine.
   *   <li>Updates the simulation of each swerve module.
   *   <li>Applies the propelling forces of the modules to the physics engine.
   *   <li>Updates the gyro simulation of the drivetrain.
   * </ul>
   */
  @Override
  public void simulationSubTick() {
    simulateChassisFrictionForce();

    simulateChassisFrictionTorque();

    simulateModulePropellingForces();

    gyroSimulation.updateSimulationSubTick(RadiansPerSecond.of(super.getAngularVelocity()));
  }

  private Translation2d previousModuleSpeedsFieldRelative = new Translation2d();

  /**
   *
   *
   * <h2>Simulates the Translational Friction Force and Applies It to the Physics Engine.</h2>
   *
   * <p>This method simulates the translational friction forces acting on the robot and applies them
   * to the physics engine. There are two components of the friction forces:
   *
   * <ul>
   *   <li>A portion of the friction force pushes the robot from its current ground speeds ({@link
   *       #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current module speeds
   *       ({@link #getModuleSpeeds()}).
   *   <li>Another portion of the friction force is the centripetal force, which occurs when the
   *       chassis changes its direction of movement.
   * </ul>
   *
   * <p>The total friction force should not exceed the tire's grip limit.
   */
  private void simulateChassisFrictionForce() {
    final ChassisSpeeds moduleSpeeds = getModuleSpeeds();

    /* The friction force that tries to bring the chassis from floor speeds to module speeds */
    final ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative =
        moduleSpeeds.minus(getDriveTrainSimulatedChassisSpeedsRobotRelative());
    final Translation2d floorAndModuleSpeedsDiffFieldRelative =
        new Translation2d(
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vxMetersPerSecond,
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vyMetersPerSecond)
            .rotateBy(getSimulatedDriveTrainPose().getRotation());
    final double FRICTION_FORCE_GAIN = 3.0;
    final Force totalGrippingForce =
            moduleSimulations[0].getGrippingForceNewtons(gravityForceOnEachModule)
                .times(moduleSimulations.length);
    final Vector2 speedsDifferenceFrictionForce =
        Vector2.create(
            Math.min(
                FRICTION_FORCE_GAIN
                    * totalGrippingForce.in(Newtons)
                    * floorAndModuleSpeedsDiffFieldRelative.getNorm(),
                totalGrippingForce.in(Newtons)),
            MapleCommonMath.getAngle(floorAndModuleSpeedsDiffFieldRelative).getRadians());

    /* the centripetal friction force during turning */
    final ChassisSpeeds moduleSpeedsFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            moduleSpeeds, getSimulatedDriveTrainPose().getRotation());
    final Rotation2d dTheta =
        MapleCommonMath.getAngle(
                GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative))
            .minus(MapleCommonMath.getAngle(previousModuleSpeedsFieldRelative));

    final double orbitalAngularVelocity = dTheta.getRadians() / SimulatedArena.getSimulationDt().in(Seconds);
    final Rotation2d centripetalForceDirection =
        MapleCommonMath.getAngle(previousModuleSpeedsFieldRelative)
            .plus(Rotation2d.fromDegrees(90));
    final Vector2 centripetalFrictionForce =
        Vector2.create(
            previousModuleSpeedsFieldRelative.getNorm()
                * orbitalAngularVelocity
                * profile.robotMass.in(Kilograms),
            centripetalForceDirection.getRadians());
    previousModuleSpeedsFieldRelative =
        GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative);

    /* apply force to physics engine */
    final Vector2
        totalFrictionForceUnlimited =
            centripetalFrictionForce.copy().add(speedsDifferenceFrictionForce),
        totalFrictionForce =
            Vector2.create(
                Math.min(totalGrippingForce.in(Newtons), totalFrictionForceUnlimited.getMagnitude()),
                totalFrictionForceUnlimited.getDirection());
    super.applyForce(totalFrictionForce);
  }

  /**
   *
   *
   * <h2>Simulates the Rotational Friction Torque and Applies It to the Physics Engine.</h2>
   *
   * <p>This method simulates the rotational friction torque acting on the robot and applies them to
   * the physics engine.
   *
   * <p>The friction torque pushes the robot from its current ground angular velocity ({@link
   * #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current modules' angular
   * velocity ({@link #getModuleSpeeds()}).
   */
  private void simulateChassisFrictionTorque() {
    final double
        desiredRotationalMotionPercent =
            Math.abs(getDesiredSpeed().omegaRadiansPerSecond / getTheoreticalMaxAngularVelocity().in(RadiansPerSecond)),
        actualRotationalMotionPercent =
            Math.abs(getAngularVelocity() / getTheoreticalMaxAngularVelocity().in(RadiansPerSecond)),
        differenceBetweenFloorSpeedAndModuleSpeed =
            getModuleSpeeds().omegaRadiansPerSecond - getAngularVelocity(),
        grippingTorqueMagnitude =
            moduleSimulations[0].getGrippingForceNewtons(gravityForceOnEachModule).in(Newtons)
                * moduleTranslations[0].getNorm()
                * moduleSimulations.length,
        FRICTION_TORQUE_GAIN = 1;

    if (actualRotationalMotionPercent < 0.01 && desiredRotationalMotionPercent < 0.02)
      super.setAngularVelocity(0);
    else
      super.applyTorque(
          Math.copySign(
              Math.min(
                  FRICTION_TORQUE_GAIN
                      * grippingTorqueMagnitude
                      * Math.abs(differenceBetweenFloorSpeedAndModuleSpeed),
                  grippingTorqueMagnitude),
              differenceBetweenFloorSpeedAndModuleSpeed));
  }

  /**
   *
   *
   * <h2>Simulates the Translational Friction Force and Applies It to the Physics Engine.</h2>
   *
   * <p>This method simulates the translational friction forces acting on the robot and applies them
   * to the physics engine. There are two components of the friction forces:
   *
   * <ul>
   *   <li>A portion of the friction force pushes the robot from its current ground speeds ({@link
   *       #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current module speeds
   *       ({@link #getModuleSpeeds()}).
   *   <li>Another portion of the friction force is the centripetal force, which occurs when the
   *       chassis changes its direction of movement.
   * </ul>
   *
   * <p>The total friction force should not exceed the tire's grip limit.
   */
  private void simulateModulePropellingForces() {
    for (int i = 0; i < moduleSimulations.length; i++) {
      final Vector2 moduleWorldPosition =
          getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));
      final Vector2 moduleForce =
          moduleSimulations[i].updateSimulationSubTickGetModuleForce(
              super.getLinearVelocity(moduleWorldPosition),
              getSimulatedDriveTrainPose().getRotation(),
              gravityForceOnEachModule);
      super.applyForce(moduleForce, moduleWorldPosition);
    }
  }

  /**
   *
   *
   * <h2>Obtains the Chassis Speeds the Modules Are Attempting to Achieve.</h2>
   *
   * <p>This method returns the desired chassis speeds that the modules are trying to reach. If the
   * robot maintains the current driving voltage and steering position for a long enough period, it
   * will achieve these speeds.
   *
   * @return the desired chassis speeds, robot-relative
   */
  private ChassisSpeeds getDesiredSpeed() {
    return kinematics.toChassisSpeeds(
        Arrays.stream(moduleSimulations)
            .map((SwerveModuleSimulation::getFreeSpinState))
            .toArray(SwerveModuleState[]::new));
  }

  /**
   *
   *
   * <h2>Obtains the Current Chassis Speeds of the Modules.</h2>
   *
   * <p>This method estimates the chassis speeds of the robot based on the swerve states of the
   * modules.
   *
   * <p><strong>Note:</strong> These speeds might not represent the actual floor speeds due to
   * potential skidding.
   *
   * @return the module speeds, robot-relative
   */
  private ChassisSpeeds getModuleSpeeds() {
    return kinematics.toChassisSpeeds(
        Arrays.stream(moduleSimulations)
            .map((SwerveModuleSimulation::getCurrentState))
            .toArray(SwerveModuleState[]::new));
  }

  public LinearVelocity getTheoreticalMaxLinearVelocity() {
    return moduleSimulations[0].getModuleTheoreticalSpeedMPS();
  }

  public LinearAcceleration getTheoreticalMaxLinearAcceleration() {
    return moduleSimulations[0].getModuleMaxAccelerationMPSsq(
        profile.robotMass, moduleSimulations.length);
  }

  public AngularVelocity getTheoreticalMaxAngularVelocity() {
    return RadiansPerSecond.of(moduleSimulations[0].getModuleTheoreticalSpeedMPS().divide(moduleTranslations[0].getNorm()).in(MetersPerSecond));
  }

  public AngularAcceleration getSwerveDriveMaxAngularAcceleration() {
    return RadiansPerSecondPerSecond.of(moduleSimulations[0].getTheoreticalPropellingForcePerModule(
            profile.robotMass, moduleSimulations.length).in(Newtons)
        * moduleTranslations[0].getNorm()
        * moduleSimulations.length
        / super.getMass().getInertia());
  }

  public SwerveModuleSimulation[] getModules() {
    return moduleSimulations;
  }
}
