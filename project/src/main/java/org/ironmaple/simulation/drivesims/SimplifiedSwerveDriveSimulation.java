package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;

import org.ironmaple.simulation.MapleMotorSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.SwerveStateProjection;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 *
 *
 * <h1>An easier way to simulate swerve drive.</h1>
 *
 * <p><a href='https://shenzhen-robotics-alliance.github.io/maple-sim/3.1_SWERVE_SIM_EZ_MODE.html'>Check Online
 * Documents</a>
 *
 * <p>This class owns and controls a {@link SwerveDriveSimulation}, running closed loops/open loops on the simulated
 * motors.
 *
 * <p>It works identically to how the real swerve simulation code.
 *
 * <p>Note: The order for swerve modules is: front-left, front-right, back-left, back-right.
 */
public class SimplifiedSwerveDriveSimulation {
    private final SwerveDriveSimulation swerveDriveSimulation;
    private final SwerveModuleSimulation[] moduleSimulations;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModuleState[] setPointsOptimized;

    /**
     *
     *
     * <h2>Default Constructor.</h2>
     *
     * <p>Constructs a simplified swerve simulation with default standard deviations for odometry & vision pose
     * estimates.
     *
     * <p>Odometry is simulated as high-frequency odometry, assuming a robot period of 0.02 seconds and an odometry
     * frequency of 250 Hz.
     *
     * @param swerveDriveSimulation the {@link SwerveDriveSimulation} to control.
     */
    public SimplifiedSwerveDriveSimulation(SwerveDriveSimulation swerveDriveSimulation) {
        this(swerveDriveSimulation, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9));
    }

    /**
     *
     *
     * <h2>Constructs an instance with given odometry standard deviations.</h2>
     *
     * <p>Constructs a simplified swerve simulation with specified standard deviations for odometry & vision pose
     * estimates.
     *
     * @param swerveDriveSimulation the {@link SwerveDriveSimulation} to control.
     * @param stateStdDevs the standard deviations for odometry encoders.
     * @param visionMeasurementStdDevs the standard deviations for vision pose estimates.
     */
    public SimplifiedSwerveDriveSimulation(
            SwerveDriveSimulation swerveDriveSimulation,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        this.swerveDriveSimulation = swerveDriveSimulation;
        this.moduleSimulations = swerveDriveSimulation.getModules();

        this.kinematics = swerveDriveSimulation.kinematics;

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getRawGyroAngle(),
            getLatestModulePositions(),
            getActualPoseInSimulationWorld(),
            stateStdDevs,
            visionMeasurementStdDevs);

        this.setPointsOptimized = new SwerveModuleState[moduleSimulations.length];
        Arrays.fill(setPointsOptimized, new SwerveModuleState());
    }

    /**
     *
     *
     * <h2>Periodic Method for Simplified Swerve Sim.</h2>
     *
     * <p>Call this method in the {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic()} of your swerve subsystem.
     *
     * <p>Updates the odometry by fetching cached inputs.
     */
    public void periodic() {
        final SwerveModulePosition[][] cachedModulePositions = getCachedModulePositions();
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            poseEstimator.updateWithTime(
                    Timer.getFPGATimestamp()
                            - SimulatedArena.getSimulationDt() * (SimulatedArena.getSimulationDt() - i),
                    swerveDriveSimulation.gyroSimulation.getCachedGyroReadings()[i],
                    cachedModulePositions[i]);
    }

    /**
     *
     *
     * <h2>Obtains the LATEST module positions measured by the encoders.</h2>
     *
     * <p>The order for swerve modules is: front-left, front-right, back-left, back-right.
     *
     * @return the module positions
     */
    public SwerveModulePosition[] getLatestModulePositions() {
        return Arrays.stream(moduleSimulations)
                .map(moduleSimulation -> new SwerveModulePosition(
                        moduleSimulation.getDriveWheelFinalPositionRad() * moduleSimulation.WHEEL_RADIUS_METERS,
                        moduleSimulation.getSteerAbsoluteFacing()))
                .toArray(SwerveModulePosition[]::new);
    }

    /**
     *
     *
     * <h2>Obtains the CACHED module positions measured by the encoders.</h2>
     *
     * <p>This simulates high-frequency odometry.
     *
     * <p>The module positions, or the value of {@link #getLatestModulePositions()} are cached in every simulation
     * sub-tick.
     *
     * <p>The array is ordered in a [timeStampIndex][moduleIndex] format.
     *
     * <p>The order for swerve modules is: front-left, front-right, back-left, back-right.
     *
     * @return the cached module positions
     */
    public SwerveModulePosition[][] getCachedModulePositions() {
        final SwerveModulePosition[][] cachedModulePositions =
                new SwerveModulePosition[SimulatedArena.getSimulationSubTicksIn1Period()][moduleSimulations.length];

        for (int moduleIndex = 0; moduleIndex < moduleSimulations.length; moduleIndex++) {
            final double[] wheelPositionRads = moduleSimulations[moduleIndex].getCachedDriveWheelFinalPositionsRad();
            final Rotation2d[] swerveModuleFacings = moduleSimulations[moduleIndex].getCachedSteerAbsolutePositions();
            for (int timeStamp = 0; timeStamp < SimulatedArena.getSimulationSubTicksIn1Period(); timeStamp++)
                cachedModulePositions[timeStamp][moduleIndex] = new SwerveModulePosition(
                        wheelPositionRads[timeStamp] * moduleSimulations[0].WHEEL_RADIUS_METERS,
                        swerveModuleFacings[timeStamp]);
        }

        return cachedModulePositions;
    }

    /**
     *
     *
     * <h2>Obtains the raw angle of the gyro.</h2>
     *
     * <p>Note that the simulated gyro also drifts/skids like real gyros, especially if the robot collides.
     *
     * <p>To obtain the facing of the robot retrieved from the odometry, use {@link #getOdometryEstimatedPose()}; to
     * obtain the actual facing of the robot, use {@link #getActualPoseInSimulationWorld()}.
     *
     * @deprecated This rotation is <strong>NOT</strong> the actual facing of the robot; it is uncalibrated and is only
     *     used for features like rotation lock.
     * @return the raw (uncalibrated) angle of the simulated gyro.
     */
    @Deprecated
    public Rotation2d getRawGyroAngle() {
        return swerveDriveSimulation.gyroSimulation.getGyroReading();
    }

    /**
     *
     *
     * <h2>Obtains the robot pose measured by the odometry.</h2>
     *
     * <p>Retrieves the pose estimated by the odometry system (and vision, if applicable).
     *
     * <p>This method wraps around {@link SwerveDrivePoseEstimator#getEstimatedPosition()}.
     *
     * <p>This represents the estimated position of the robot.
     *
     * <p>Note that this estimation includes realistic simulations of measurement errors due to skidding and odometry
     * drift.
     *
     * <p>To obtain the ACTUAL pose of the robot, with no measurement errors, use
     * {@link #getActualPoseInSimulationWorld()}.
     */
    public Pose2d getOdometryEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     *
     *
     * <h2>Resets the odometry to a specified position.</h2>
     *
     * <p>This method wraps around {@link SwerveDrivePoseEstimator#resetPosition(Rotation2d, SwerveModulePosition[], Pose2d)}.
     *
     * <p>It resets the position of the pose estimator to the given pose.
     *
     * @param pose The position on the field where the robot is located.
     */
    public void resetOdometry(Pose2d pose) {
        this.poseEstimator.resetPosition(getRawGyroAngle(), getLatestModulePositions(), pose);
    }

    /**
     *
     *
     * <h2>Adds a vision estimation to the pose estimator.</h2>
     *
     * <p>This method wraps around {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
     *
     * <p>Adds a vision measurement to the Kalman Filter, correcting the odometry pose estimate while accounting for
     * measurement noise.
     *
     * @param robotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timeStampSeconds The timestamp of the vision measurement, in seconds.
     */
    public void addVisionEstimation(Pose2d robotPoseMeters, double timeStampSeconds) {
        this.poseEstimator.addVisionMeasurement(robotPoseMeters, timeStampSeconds);
    }

    /**
     *
     *
     * <h2>Adds a vision estimation to the pose estimator.</h2>
     *
     * <p>This method wraps around {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     *
     * <p>Adds a vision measurement to the Kalman Filter, correcting the odometry pose estimate while accounting for
     * measurement noise.
     *
     * @param robotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timeStampSeconds The timestamp of the vision measurement, in seconds.
     * @param measurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y position in
     *     meters, and heading in radians). Increase these values to reduce the trust in the vision pose measurement.
     */
    public void addVisionEstimation(
            Pose2d robotPoseMeters, double timeStampSeconds, Matrix<N3, N1> measurementStdDevs) {
        this.poseEstimator.addVisionMeasurement(robotPoseMeters, timeStampSeconds, measurementStdDevs);
    }

    /**
     *
     *
     * <h2>Runs chassis speeds on the simulated swerve drive.</h2>
     *
     * <p>Runs the specified chassis speeds, either robot-centric or field-centric.
     *
     * @param chassisSpeeds The speeds to run, in either robot-centric or field-centric coordinates.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of rotation at one
     *     corner of the robot and provide a chassis speed that has only a dtheta component, the robot will rotate
     *     around that corner.
     * @param fieldCentricDrive Whether to execute field-centric drive with the provided speed.
     * @param discretizeSpeeds Whether to apply {@link ChassisSpeeds#discretize(ChassisSpeeds, double)} to the provided
     *     speed.
     */
    public void runChassisSpeeds(
            ChassisSpeeds chassisSpeeds,
            Translation2d centerOfRotationMeters,
            boolean fieldCentricDrive,
            boolean discretizeSpeeds) {
        if (fieldCentricDrive)
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds, getOdometryEstimatedPose().getRotation());
        if (discretizeSpeeds)
            chassisSpeeds = ChassisSpeeds.discretize(
                    chassisSpeeds, SimulatedArena.getSimulationDt() * SimulatedArena.getSimulationSubTicksIn1Period());
        final SwerveModuleState[] setPoints = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotationMeters);
        for (int i = 0; i < moduleSimulations.length; i++)
            setPointsOptimized[i] = optimizeAndRunModuleState(moduleSimulations[i], setPoints[i]);
    }

    /**
     *
     *
     * <h2>Obtain the MEASURED swerve states.</h2>
     *
     * <p>The order of the swerve modules is: front-left, front-right, back-left, back-right.
     *
     * @return The actual measured swerve states of the simulated swerve.
     */
    public SwerveModuleState[] getMeasuredStates() {
        return Arrays.stream(moduleSimulations)
                .map(SwerveModuleSimulation::getCurrentState)
                .toArray(SwerveModuleState[]::new);
    }

    /**
     *
     *
     * <h2>Obtain the optimized SETPOINTS of the swerve.</h2>
     *
     * <p>The setpoints are calculated using {@link SwerveDriveKinematics#toSwerveModuleStates(ChassisSpeeds)} in the
     * most recent call to {@link #runChassisSpeeds(ChassisSpeeds, Translation2d, boolean, boolean)}.
     *
     * <p>The setpoints are optimized using {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}.
     *
     * <p>The order of the swerve modules is: front-left, front-right, back-left, back-right.
     *
     * @return The optimized setpoints of the swerve, calculated during the last chassis speed run.
     */
    public SwerveModuleState[] getSetPointsOptimized() {
        return setPointsOptimized;
    }

    /**
     *
     *
     * <h2>Obtain the field-relative chassis speeds measured from the encoders.</h2>
     *
     * <p>The speeds are measured from the simulated swerve modules.
     *
     * @param useGyroForAngularVelocity Whether to use the gyro for a more accurate angular velocity measurement.
     * @return The measured chassis speeds, <strong>field-relative</strong>.
     */
    public ChassisSpeeds getMeasuredSpeedsFieldRelative(boolean useGyroForAngularVelocity) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getMeasuredSpeedsRobotRelative(useGyroForAngularVelocity),
                getOdometryEstimatedPose().getRotation());
    }

    /**
     *
     *
     * <h2>Obtain the robot-relative chassis speeds measured from the encoders.</h2>
     *
     * <p>The speeds are measured from the simulated swerve modules.
     *
     * @param useGyroForAngularVelocity Whether to use the gyro for a more accurate angular velocity measurement.
     * @return The measured chassis speeds, <strong>robot-relative</strong>.
     */
    public ChassisSpeeds getMeasuredSpeedsRobotRelative(boolean useGyroForAngularVelocity) {
        final ChassisSpeeds swerveSpeeds = kinematics.toChassisSpeeds(getMeasuredStates());
        return new ChassisSpeeds(
                swerveSpeeds.vxMetersPerSecond,
                swerveSpeeds.vyMetersPerSecond,
                useGyroForAngularVelocity
                        ? swerveDriveSimulation.gyroSimulation.getMeasuredAngularVelocityRadPerSec()
                        : swerveSpeeds.omegaRadiansPerSecond);
    }

    /**
     *
     *
     * <h2>Obtain the {@link SwerveDriveSimulation} object controlled by this simplified swerve simulation.</h2>
     *
     * @return The swerve drive simulation.
     */
    public SwerveDriveSimulation getDriveTrainSimulation() {
        return this.swerveDriveSimulation;
    }

    /**
     *
     *
     * <h2>Obtain the ACTUAL robot pose.</h2>
     *
     * <p>Obtains the ACTUAL robot pose with zero measurement error.
     *
     * <p>To obtain the pose calculated by odometry (where the robot thinks it is), use
     * {@link #getOdometryEstimatedPose()}.
     */
    public Pose2d getActualPoseInSimulationWorld() {
        return swerveDriveSimulation.getSimulatedDriveTrainPose();
    }

    /**
     *
     *
     * <h2>Get the ACTUAL field-relative chassis speeds of the robot.</h2>
     *
     * <p>Wraps around {@link SwerveDriveSimulation#getDriveTrainSimulatedChassisSpeedsFieldRelative()}.
     *
     * @return the actual chassis speeds in the simulation world, <strong>field-relative</strong>
     */
    public ChassisSpeeds getActualSpeedsFieldRelative() {
        return this.swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    /**
     *
     *
     * <h2>Get the ACTUAL robot-relative chassis speeds of the robot.</h2>
     *
     * <p>Wraps around {@link SwerveDriveSimulation#getDriveTrainSimulatedChassisSpeedsRobotRelative()}.
     *
     * @return the actual chassis speeds in the simulation world, <strong>robot-relative</strong>
     */
    public ChassisSpeeds getActualSpeedsRobotRelative() {
        return this.swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
    }

    /**
     *
     *
     * <h2>Teleport the robot to a specified location on the simulated field.</h2>
     *
     * <p>This method moves the drivetrain instantly to the specified location on the field, bypassing any obstacles in
     * its path.
     *
     * <p>Wraps around {@link SwerveDriveSimulation#setSimulationWorldPose(Pose2d)}.
     *
     * @param robotPose the pose of the robot to teleport to
     */
    public void setSimulationWorldPose(Pose2d robotPose) {
        this.swerveDriveSimulation.setSimulationWorldPose(robotPose);
    }

    /**
     *
     *
     * <h2>Runs the control loops for swerve states on a simulated module.</h2>
     *
     * <p>Optimizes the set-point using {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}.
     *
     * <p>Executes a closed-loop control on the swerve module.
     *
     * @param moduleSimulation the simulated swerve module to run the control loops on
     * @param setPoint the desired state to optimize and apply
     * @return the optimized swerve module state after control execution
     */
    private SwerveModuleState optimizeAndRunModuleState(
        SwerveModuleSimulation moduleSimulation, SwerveModuleState setPoint) {
        setPoint.optimize(moduleSimulation.getSteerAbsoluteFacing());
        final double cosProjectedSpeedMPS =
            SwerveStateProjection.project(setPoint, moduleSimulation.getSteerAbsoluteFacing()),

        driveMotorVelocitySetPointRadPerSec =
                cosProjectedSpeedMPS / moduleSimulation.WHEEL_RADIUS_METERS * moduleSimulation.DRIVE_GEAR_RATIO;

        moduleSimulation.requestSteerOutput(Radians.of(setPoint.angle.getRadians()));
        moduleSimulation.requestDriveOutput(RadiansPerSecond.of(driveMotorVelocitySetPointRadPerSec));
        return setPoint;
    }

    /**
     *
     *
     * <h2>Obtains the maximum linear velocity of the swerve modules.</h2>
     *
     * <p>This method retrieves the theoretical maximum linear velocity of the swerve module in meters per second.
     *
     * @return the maximum linear velocity in meters per second.
     */
    public double getMaximumLinearVelocityMetersPerSecond() {
        return moduleSimulations[0].getModuleTheoreticalSpeedMPS();
    }

    /**
     *
     *
     * <h2>Obtains the maximum linear acceleration of the swerve modules.</h2>
     *
     * <p>This method calculates the maximum linear acceleration of the swerve module in meters per second squared.
     *
     * @return the maximum linear acceleration in meters per second squared.
     */
    public double getMaximumLinearAccelerationMetersPerSecondSquare() {
        return moduleSimulations[0].getModuleMaxAccelerationMPSsq(
                swerveDriveSimulation.config.robotMassKg, moduleSimulations.length);
    }

    /**
     *
     *
     * <h2>Obtains the track width of the swerve drive.</h2>
     *
     * <p>This method calculates the total track width of the robot in meters, which is the distance between the left
     * and right swerve modules.
     *
     * @return the track width in meters.
     */
    public double getTrackWidthYMeters() {
        return swerveDriveSimulation.moduleTranslations[0].getY() * 2;
    }

    /**
     *
     *
     * <h2>Obtains the track length of the swerve drive.</h2>
     *
     * <p>This method calculates the total track length of the robot in meters, which is the distance between the front
     * and back swerve modules.
     *
     * @return the track length in meters.
     */
    public double getTrackLengthXMeters() {
        return swerveDriveSimulation.moduleTranslations[0].getX() * 2;
    }

    /**
     *
     *
     * <h2>Obtains the drive base radius of the swerve drive.</h2>
     *
     * <p>This method calculates the radius of the drive base, which is the distance from the center of rotation to the
     * swerve modules, in meters.
     *
     * @return the drive base radius in meters.
     */
    public double getDriveBaseRadiusMeters() {
        return swerveDriveSimulation.moduleTranslations[0].getNorm();
    }

    /**
     *
     *
     * <h2>Obtains the maximum angular velocity of the swerve drive.</h2>
     *
     * <p>This method calculates the maximum angular velocity of the robot based on the maximum linear velocity and
     * drive base radius, in radians per second.
     *
     * @return the maximum angular velocity in radians per second.
     */
    public double getMaximumAngularVelocityRadPerSec() {
        return getMaximumLinearVelocityMetersPerSecond() / getDriveBaseRadiusMeters();
    }

    /**
     *
     *
     * <h2>Obtains the maximum angular acceleration of the swerve drive.</h2>
     *
     * <p>This method calculates the maximum angular acceleration of the robot based on the maximum propelling torque
     * and the robot's moment of inertia, in radians per second squared.
     *
     * @return the maximum angular acceleration in radians per second squared.
     */
    public double getMaximumAngularAccelerationRadPerSec() {
        final double
                maxPropellingForce =
                        moduleSimulations[0].getTheoreticalPropellingForcePerModule(
                                swerveDriveSimulation.config.robotMassKg, moduleSimulations.length),
                maxPropellingTorque = maxPropellingForce * getDriveBaseRadiusMeters();
        // angular acc = torque / inertia
        return maxPropellingTorque / swerveDriveSimulation.getMass().getInertia();
    }

    /**
     *
     *
     * <h2>Configures the swerve drive simulation with a PID controller for steering.</h2>
     *
     * <p>This method sets the PID controller used for the steering closed-loop control of the swerve drive.
     *
     * @param steerHeadingCloseLoop the PID controller for steering control.
     * @param inputUnit the unit used to measure the steer angle with
     * @param useVoltage use voltage-based control for the motor
     * @return the current instance of {@link SimplifiedSwerveDriveSimulation} for method chaining.
     */
    public SimplifiedSwerveDriveSimulation withSteerPID(PIDController steerHeadingCloseLoop, AngleUnit inputUnit, boolean useVoltage) {
        for (int i = 0; i < 4; i++) {
            moduleSimulations[i] = moduleSimulations[i].withSteerPID(
                steerHeadingCloseLoop.getP(),
                steerHeadingCloseLoop.getD(),
                inputUnit,
                useVoltage
            );
        }
        return this;
    }

    /**
     *
     *
     * <h2>Configures the swerve drive simulation with a PID controller for steering.</h2>
     *
     * <p>This method sets the PID controller used for the steering closed-loop control of the swerve drive.
     *
     * @param driveCloseLoop the PID controller for drive control.
     * @param inputUnit the unit used to measure the steer angle with
     * @param useVoltage use voltage-based control for the motor
     * @return the current instance of {@link SimplifiedSwerveDriveSimulation} for method chaining.
     */
    public SimplifiedSwerveDriveSimulation withDrivePID(PIDController driveCloseLoop, AngularVelocityUnit inputUnit, boolean useVoltage) {
        for (int i = 0; i < 4; i++) {
            moduleSimulations[i] = moduleSimulations[i].withDrivePID(
                driveCloseLoop.getP(),
                inputUnit,
                useVoltage
            );
        }
        return this;
    }

    /**
     *
     *
     * <h2>Configures the swerve drive simulation with a PID controller for steering.</h2>
     *
     * <p>This method sets the PID controller used for the steering closed-loop control of the swerve drive.
     *
     * @param driveFeedforward the feedforward for drive control.
     * @param inputUnit the unit used to measure the steer angle with
     * @return the current instance of {@link SimplifiedSwerveDriveSimulation} for method chaining.
     */
    public SimplifiedSwerveDriveSimulation withDriveFeedForward(SimpleMotorFeedforward driveFeedforward, AngularVelocityUnit inputUnit) {
        for (int i = 0; i < 4; i++) {
            moduleSimulations[i] = moduleSimulations[i].withDriveFeedforward(
                driveFeedforward.getKs(),
                driveFeedforward.getKv(),
                driveFeedforward.getKa(),
                inputUnit
            );
        }
        return this;
    }
}
