package org.ironmaple.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * <h2>An abstraction of swerve drive subsystems.</h2>
 *
 * <p>In order to use {@link org.ironmaple.simulation.drivesims.SimplifiedSwerveDriveSimulation} during simulation, you should need to implement this interface in your swerve drive subsystem.</p>
 * <p>Then, use this interface in all your drive commands, and configure the auto-builder using APIs defined in this class.</p>
 * */
public interface AbstractSwerveDriveSubsystem extends Subsystem {
  Rotation2d getRawGyroAngle();

  Pose2d getEstimatedOdometryPose();

  void addVisionEstimation(Pose2d robotPoseMeters, double timeStampSeconds);

  void addVisionEstimation(
      Pose2d robotPoseMeters, double timeStampSeconds, Matrix<N3, N1> measurementStdDevs);

  default void runDriverStationCentricSpeeds(
      ChassisSpeeds driverStationCentricSpeeds, double robotPeriodSeconds) {
    final ChassisSpeeds robotCentricSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driverStationCentricSpeeds,
            getEstimatedOdometryPose()
                .getRotation()
                .minus(FieldMirroringUtils.getCurrentAllianceDriverStationFacing()));
    runChassisSpeeds(ChassisSpeeds.discretize(robotCentricSpeeds, robotPeriodSeconds));
  }

  default void runChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    runChassisSpeeds(chassisSpeeds, new Translation2d());
  }

  void runChassisSpeeds(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters);

  SwerveModuleState[] getMeasuredStates();

  SwerveModuleState[] getSetPointsOptimized();

  ChassisSpeeds getMeasuredSpeedsFieldRelative();

  ChassisSpeeds getMeasuredSpeedsRobotRelative();

  double getMaximumLinearVelocity();

  double getMaximumLinearAcceleration();

  double getDriveBaseRadius();
}
