package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.ironmaple.utils.mathutils.MapleCommonMath.constrainMagnitude;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h1>Simulates a Simplified Holonomic Drivetrain.</h1>
 *
 * <p>This class simulates a simplified holonomic drivetrain with collision space, allowing the
 * drivetrain to drive on the field.
 *
 * <p>It is suitable for simulating AI cycle bots or opponent defense bots.
 *
 * <p>However, this drivetrain simulation is simplified and not entirely realistic—while it
 * accelerates smoothly, the kinematics are not fully accurate.
 *
 * <p>For a more realistic simulation of your robot's drivetrain, use {@link SwerveDriveSimulation}
 * instead.
 */
public class SimplifiedHolonomicDriveSimulation extends AbstractDriveTrainSimulation
    implements Subsystem {
  /**
   *
   *
   * <h2>Creates a Simplified Holonomic Drivetrain Simulation.</h2>
   *
   * @param profile the profile containing all the configuration settings for the drivetrain, see
   *     {@link
   *     org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation.DriveTrainSimulationProfile}
   * @param initialPoseOnField the initial pose of the drivetrain in the simulation world
   */
  public SimplifiedHolonomicDriveSimulation(
      DriveTrainSimulationProfile profile, Pose2d initialPoseOnField) {
    super(profile, initialPoseOnField);
  }

  private ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds();

  /**
   *
   *
   * <h2>Runs Chassis Speeds on This Simulated Chassis.</h2>
   *
   * <p>This method sets the <strong>DESIRED</strong> robot-relative chassis speeds on the simulated
   * drivetrain.
   *
   * <p>Propelling forces will be applied to the chassis to help it accelerate smoothly to the
   * desired speeds.
   *
   * <p>This method is different from {@link
   * AbstractDriveTrainSimulation#setRobotSpeeds(ChassisSpeeds)}, which jumps to the speed
   * <strong>Instantaneously</strong>.
   *
   * @param speeds the desired robot-relative speeds, represented as {@link ChassisSpeeds}
   */
  public void runChassisSpeeds(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) desiredFieldRelativeSpeeds = speeds;
    else
      desiredFieldRelativeSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getSimulatedDriveTrainPose().getRotation());
  }

  /**
   *
   *
   * <h2>Update this chassis simulation.</h2>
   */
  @Override
  public void simulationSubTick() {
    simulateChassisBehaviorWithFieldRelativeSpeeds(desiredFieldRelativeSpeeds);
  }

  /**
   *
   *
   * <h2>Simulates the Chassis Behavior with Field-Relative Speeds.</h2>
   *
   * <p>This method updates the simulation with the given desired field-relative chassis speeds. It
   * simulates both the linear force and rotational torque due to friction and propulsion.
   *
   * @param desiredChassisSpeedsFieldRelative the desired chassis speeds relative to the field,
   *     represented as {@link ChassisSpeeds}
   */
  private void simulateChassisBehaviorWithFieldRelativeSpeeds(
      ChassisSpeeds desiredChassisSpeedsFieldRelative) {
    super.setAtRest(false);

    final Vector2 desiredLinearMotionPercent =
        GeometryConvertor.toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative)
            .multiply(1.0 / profile.maxLinearVelocity.in(MetersPerSecond));
    simulateChassisTranslationalBehavior(
        Vector2.create(
            constrainMagnitude(desiredLinearMotionPercent.getMagnitude(), 1),
            desiredLinearMotionPercent.getDirection()));

    final double desiredRotationalMotionPercent =
        desiredChassisSpeedsFieldRelative.omegaRadiansPerSecond / profile.maxAngularVelocity.in(RadiansPerSecond);
    simulateChassisRotationalBehavior(constrainMagnitude(desiredRotationalMotionPercent, 1));
  }

  /**
   *
   *
   * <h1>Simulates the Translational Behavior of the Chassis.</h1>
   *
   * <p>This method simulates the translational behavior of the chassis based on the desired
   * percentage of linear motion the chassis is attempting to achieve.
   *
   * @param desiredLinearMotionPercent the percentage of the linear motion the chassis is trying to
   *     reach, represented as a {@link Vector2}
   */
  private void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
    final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
    final Vector2 forceVec =
        desiredLinearMotionPercent
            .copy()
            .multiply(this.profile.robotMass.times(this.profile.maxLinearAcceleration).in(Newtons));

    if (robotRequestedToMoveLinearly) super.applyForce(new Force(forceVec));
    else simulateChassisLinearFriction();
  }

  /**
   *
   *
   * <h2>Simulates the Rotational Behavior of the Chassis.</h2>
   *
   * <p>This method simulates the rotational behavior of the chassis based on the desired percentage
   * of rotational motion the chassis is attempting to achieve.
   *
   * @param desiredRotationalMotionPercent the percentage of the rotational motion the chassis is
   *     trying to reach, represented as a double
   */
  private void simulateChassisRotationalBehavior(double desiredRotationalMotionPercent) {
    final double maximumTorque = this.profile.maxAngularAcceleration.in(RadiansPerSecondPerSecond) * super.getMass().getInertia();
    super.applyTorque(desiredRotationalMotionPercent * maximumTorque);
    simulateChassisAngularFriction(desiredRotationalMotionPercent);
  }

  public Command followTrajectory(
      Trajectory trajectory,
      Rotation2d startingRotation,
      Rotation2d endingRotation,
      boolean teleportToStartingPose) {
    final Timer trajectoryTimer = new Timer();
    final HolonomicDriveController driveController =
        new HolonomicDriveController(
            new PIDController(5.0, 0, 0.02),
            new PIDController(5.0, 0, 0.02),
            new ProfiledPIDController(
                5.0,
                0,
                0.02,
                new TrapezoidProfile.Constraints(
                    profile.maxAngularVelocity, profile.maxAngularAcceleration)));
    final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(Commands.runOnce(trajectoryTimer::start));
    if (teleportToStartingPose)
      commandGroup.addCommands(
          Commands.runOnce(
              () ->
                  setSimulationWorldPose(
                      new Pose2d(trajectory.getInitialPose().getTranslation(), startingRotation))));
    commandGroup.addCommands(
        Commands.run(
            () ->
                this.runChassisSpeeds(
                    driveController.calculate(
                        getSimulatedDriveTrainPose(),
                        trajectory.sample(trajectoryTimer.get()),
                        startingRotation.interpolate(
                            endingRotation,
                            trajectoryTimer.get() / trajectory.getTotalTimeSeconds())),
                    false)));
    return commandGroup;
  }
}
