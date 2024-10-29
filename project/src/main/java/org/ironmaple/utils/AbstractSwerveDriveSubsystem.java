package org.ironmaple.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 *
 *
 * <h2>An abstraction of swerve drive subsystems.</h2>
 *
 * <p>In order to use {@link org.ironmaple.simulation.drivesims.SimplifiedSwerveDriveSimulation}
 * during simulation, you should need to implement this interface in your swerve drive subsystem.
 *
 * <p>Then, use this interface in all your drive commands, and configure the auto-builder using APIs
 * defined in this class.
 */
public interface AbstractSwerveDriveSubsystem extends Subsystem {
  Rotation2d getRawGyroAngle();

  Pose2d getEstimatedOdometryPose();

  void resetOdometry(Pose2d pose);

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

  default void sendYAGSLSwerveDriveWidgetToDashboard(String key) {
    SmartDashboard.putData(
        key + "SetPointsOptimized",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> getSetPointsOptimized()[0].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> getSetPointsOptimized()[0].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> getSetPointsOptimized()[1].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> getSetPointsOptimized()[1].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> getSetPointsOptimized()[2].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> getSetPointsOptimized()[2].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> getSetPointsOptimized()[3].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> getSetPointsOptimized()[3].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> getEstimatedOdometryPose().getRotation().getRadians(), null);
        });

    SmartDashboard.putData(
        key + "Measured",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> getMeasuredStates()[0].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> getMeasuredStates()[0].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> getMeasuredStates()[1].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> getMeasuredStates()[1].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> getMeasuredStates()[2].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> getMeasuredStates()[2].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> getMeasuredStates()[3].angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> getMeasuredStates()[3].speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> getEstimatedOdometryPose().getRotation().getRadians(), null);
        });
  }
}
