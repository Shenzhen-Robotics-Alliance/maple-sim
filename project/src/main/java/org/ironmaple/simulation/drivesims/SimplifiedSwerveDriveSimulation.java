package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.SwerveStateProjection;

/**
 *
 *
 * <h1>An easier way to simulate swerve drive.</h1>
 *
 * <p><a
 * href='https://shenzhen-robotics-alliance.github.io/maple-sim/3.1_SWERVE_SIM_EZ_MODE.html'>Check
 * Online Documents</a>
 *
 * <p>This class owns and controls a {@link SwerveDriveSimulation}, running close loops/open loops
 * on the simulated motors.
 *
 * <p>It works identically to how the real swerve simulation code.
 *
 * <p>This class
 */
public class SimplifiedSwerveDriveSimulation {
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final SwerveModuleSimulation[] moduleSimulations;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveModuleState[] setPointsOptimized;

  private final PIDController steerHeadingCloseLoop;
  private final PIDController driveCloseLoop;
  private final SimpleMotorFeedforward driveOpenLoop;

  public SimplifiedSwerveDriveSimulation(SwerveDriveSimulation swerveDriveSimulation) {
    this(swerveDriveSimulation, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9));
  }

  public SimplifiedSwerveDriveSimulation(
      SwerveDriveSimulation swerveDriveSimulation,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.moduleSimulations = swerveDriveSimulation.getModules();

    this.steerHeadingCloseLoop = new PIDController(5.0, 0, 0);
    this.steerHeadingCloseLoop.enableContinuousInput(-Math.PI, Math.PI);
    this.driveCloseLoop = new PIDController(0, 0, 0);
    this.driveOpenLoop =
        new SimpleMotorFeedforward(
            moduleSimulations[0].DRIVE_FRICTION_VOLTAGE,
            moduleSimulations[0].DRIVE_MOTOR.nominalVoltageVolts
                / moduleSimulations[0].DRIVE_MOTOR.freeSpeedRadPerSec);
    this.kinematics = swerveDriveSimulation.kinematics;

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getRawGyroAngle(),
            getLatestModulePositions(),
            getActualPoseInSimulationWorld(),
            stateStdDevs,
            visionMeasurementStdDevs);

    this.setPointsOptimized = new SwerveModuleState[moduleSimulations.length];
    Arrays.fill(setPointsOptimized, new SwerveModuleState());
  }

  public Rotation2d getRawGyroAngle() {
    return swerveDriveSimulation.gyroSimulation.getGyroReading();
  }

  public Pose2d getActualPoseInSimulationWorld() {
    return swerveDriveSimulation.getSimulatedDriveTrainPose();
  }

  public Pose2d getEstimatedOdometryPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    this.poseEstimator.resetPosition(getRawGyroAngle(), getLatestModulePositions(), pose);
  }

  public void addVisionEstimation(Pose2d robotPoseMeters, double timeStampSeconds) {
    this.poseEstimator.addVisionMeasurement(robotPoseMeters, timeStampSeconds);
  }

  public void addVisionEstimation(
      Pose2d robotPoseMeters, double timeStampSeconds, Matrix<N3, N1> measurementStdDevs) {
    this.poseEstimator.addVisionMeasurement(robotPoseMeters, timeStampSeconds, measurementStdDevs);
  }

  public void runChassisSpeeds(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
    final SwerveModuleState[] setPoints =
        kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotationMeters);
    for (int i = 0; i < moduleSimulations.length; i++)
      setPointsOptimized[i] = optimizeAndRunModuleState(moduleSimulations[i], setPoints[i]);
  }

  public SwerveModuleState[] getMeasuredStates() {
    return Arrays.stream(moduleSimulations)
        .map(SwerveModuleSimulation::getCurrentState)
        .toArray(SwerveModuleState[]::new);
  }

  public SwerveModuleState[] getSetPointsOptimized() {
    return setPointsOptimized;
  }

  public ChassisSpeeds getMeasuredSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getMeasuredSpeedsRobotRelative(), getEstimatedOdometryPose().getRotation());
  }

  public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
    return kinematics.toChassisSpeeds(getMeasuredStates());
  }

  public double getMaximumLinearVelocityMetersPerSecond() {
    return moduleSimulations[0].getModuleTheoreticalSpeedMPS();
  }

  public double getMaximumLinearAccelerationMetersPerSecondSquare() {
    return moduleSimulations[0].getModuleMaxAccelerationMPSsq(
        swerveDriveSimulation.config.robotMassKg, moduleSimulations.length);
  }

  public double getTrackWidthYMeters() {
    return swerveDriveSimulation.moduleTranslations[0].getY() * 2;
  }

  public double getTrackLengthXMeters() {
    return swerveDriveSimulation.moduleTranslations[0].getX() * 2;
  }

  public double getDriveBaseRadiusMeters() {
    return swerveDriveSimulation.moduleTranslations[0].getNorm();
  }

  public double getMaximumAngularVelocityRadPerSec() {
    return getMaximumLinearVelocityMetersPerSecond() / getDriveBaseRadiusMeters();
  }

  public double getMaximumAngularAccelerationRadPerSec() {
    final double
        maxPropellingForce =
            moduleSimulations[0].getTheoreticalPropellingForcePerModule(
                swerveDriveSimulation.config.robotMassKg, moduleSimulations.length),
        maxPropellingTorque = maxPropellingForce * getDriveBaseRadiusMeters();
    // angular acc = torque / inertia
    return maxPropellingTorque / swerveDriveSimulation.getMass().getInertia();
  }

  public SwerveDriveSimulation getDriveTrainSimulation() {
    return this.swerveDriveSimulation;
  }

  public ChassisSpeeds getActualSpeedsFieldRelative() {
    return this.swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
  }

  public ChassisSpeeds getActualSpeedsRobotRelative() {
    return this.swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  public void setSimulationWorldPose(Pose2d robotPose) {
    this.swerveDriveSimulation.setSimulationWorldPose(robotPose);
  }

  public void periodic() {
    final SwerveModulePosition[][] cachedModulePositions = getCachedModulePositions();
    for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
      poseEstimator.updateWithTime(
          Timer.getFPGATimestamp()
              - SimulatedArena.getSimulationDt() * (SimulatedArena.getSimulationDt() - i),
          swerveDriveSimulation.gyroSimulation.getCachedGyroReadings()[i],
          cachedModulePositions[i]);
  }

  public SwerveModulePosition[] getLatestModulePositions() {
    return Arrays.stream(moduleSimulations)
        .map(
            moduleSimulation ->
                new SwerveModulePosition(
                    moduleSimulation.getDriveWheelFinalPositionRad()
                        * moduleSimulation.WHEEL_RADIUS_METERS,
                    moduleSimulation.getSteerAbsoluteFacing()))
        .toArray(SwerveModulePosition[]::new);
  }

  public SwerveModulePosition[][] getCachedModulePositions() {
    final SwerveModulePosition[][] cachedModulePositions =
        new SwerveModulePosition[SimulatedArena.getSimulationSubTicksIn1Period()]
            [moduleSimulations.length];

    for (int moduleIndex = 0; moduleIndex < moduleSimulations.length; moduleIndex++) {
      final double[] wheelPositionRads =
          moduleSimulations[moduleIndex].getCachedDriveWheelFinalPositionsRad();
      final Rotation2d[] swerveModuleFacings =
          moduleSimulations[moduleIndex].getCachedSteerAbsolutePositions();
      for (int timeStamp = 0;
          timeStamp < SimulatedArena.getSimulationSubTicksIn1Period();
          timeStamp++)
        cachedModulePositions[timeStamp][moduleIndex] =
            new SwerveModulePosition(
                wheelPositionRads[timeStamp] * moduleSimulations[0].WHEEL_RADIUS_METERS,
                swerveModuleFacings[timeStamp]);
    }

    return cachedModulePositions;
  }

  public SwerveModuleState optimizeAndRunModuleState(
      SwerveModuleSimulation moduleSimulation, SwerveModuleState setPoint) {
    setPoint = SwerveModuleState.optimize(setPoint, moduleSimulation.getSteerAbsoluteFacing());
    final double
        cosProjectedSpeedMPS =
            SwerveStateProjection.project(setPoint, moduleSimulation.getSteerAbsoluteFacing()),
        driveMotorVelocitySetPointRadPerSec =
            cosProjectedSpeedMPS
                / moduleSimulation.WHEEL_RADIUS_METERS
                * moduleSimulation.DRIVE_GEAR_RATIO;
    final double
        driveFeedForwardVoltage = driveOpenLoop.calculate(driveMotorVelocitySetPointRadPerSec),
        driveFeedBackVoltage =
            driveCloseLoop.calculate(
                moduleSimulation.getDriveWheelFinalSpeedRadPerSec(),
                driveMotorVelocitySetPointRadPerSec),
        steerFeedBackVoltage =
            steerHeadingCloseLoop.calculate(
                moduleSimulation.getSteerAbsoluteFacing().getRadians(),
                setPoint.angle.getRadians());

    moduleSimulation.requestDriveVoltageOut(driveFeedForwardVoltage + driveFeedBackVoltage);
    moduleSimulation.requestSteerVoltageOut(steerFeedBackVoltage);
    return setPoint;
  }
}
