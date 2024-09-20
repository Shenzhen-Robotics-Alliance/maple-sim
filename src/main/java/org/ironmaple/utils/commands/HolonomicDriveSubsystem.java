// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/ under MIT License

package org.ironmaple.utils.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.ironmaple.utils.FieldMirroringUtils;
import org.ironmaple.utils.mathutils.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

/**
 * <p>common interface for a holonomic drive</p>
 * <p>field-centric drive logic and path-palnner configurations are written here, since these codes are used for both AI-Robots in the simulation and our main robot</p>
 * */
public interface HolonomicDriveSubsystem extends Subsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     * @param speeds a discrete chassis speed, robot-centric
     * */
    void runRawChassisSpeeds(ChassisSpeeds speeds);

    /**
     * @return the estimate pose from the odometry.
     */
    Pose2d getPose();

    /**
     * @return the estimate robot facing from the odometry.
     */
    default Rotation2d getFacing() {return getPose().getRotation(); }

    /**
     * gets the raw gyro reading if exists;
     * otherwise, returns the facing of the robot estimated by the odometry
     * @return the raw gyro reading of the gyro, not calibrated
     * */
    default Rotation2d getRawGyroYaw() {return getFacing(); }

    /**
     * Resets the current odometry Pose to a given Pose
     * @param currentPose the new pose yielding where the robot actually is
     */
    void setPose(Pose2d currentPose);

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     * @param measurementStdDevs the standard deviation of the measurement
     */
    default void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {}

    /**
     * get the velocity measured by the encoders
     * @return the robot-relative velocities of the chassis
     * */
    ChassisSpeeds getMeasuredChassisSpeedsRobotRelative();

    /**
     * get the velocity measured by the encoders
     * @return the field-relative velocities of the chassis
     * */
    default ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeedsRobotRelative(), getFacing());
    }

    double getChassisMaxLinearVelocityMetersPerSec();
    double getChassisMaxAccelerationMetersPerSecSq();
    double getChassisMaxAngularVelocity();
    double getChassisMaxAngularAccelerationRadPerSecSq();

    default PathConstraints getChassisConstrains(double speedMultiplier) {
        return new PathConstraints(
                getChassisMaxLinearVelocityMetersPerSec() * speedMultiplier,
                getChassisMaxAccelerationMetersPerSecSq() ,
                getChassisMaxAngularVelocity() * speedMultiplier,
                getChassisMaxAngularAccelerationRadPerSecSq()
        );
    }

    /**
     * runs a driverstation-centric ChassisSpeeds
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     * */
    default void runDriverStationCentricChassisSpeeds(ChassisSpeeds driverStationCentricSpeeds) {
        runDriverStationCentricChassisSpeeds(driverStationCentricSpeeds, FieldMirroringUtils.getCurrentAllianceDriverStationFacing());
    }

    /**
     * runs a driverstation-centric ChassisSpeeds
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     * */
    default void runDriverStationCentricChassisSpeeds(ChassisSpeeds driverStationCentricSpeeds, Rotation2d driverStationFacing) {
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverStationCentricSpeeds,
                getPose().getRotation().minus(driverStationFacing)
        ));
    }

    /**
     * runs a field-centric ChassisSpeeds
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     * */
    default void runFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricSpeeds) {
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds,
                getPose().getRotation()
        ));
    }

    /**
     * Stops the chassis, making it still.
     * On default, it runs an empty speed.
     * */
    default void stop() {
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * runs a ChassisSpeeds, pre-processed with ChassisSpeeds.discretize()
     * @param speeds a continuous chassis speed, robot-centric
     * */
    default void runRobotCentricChassisSpeeds(ChassisSpeeds speeds) {
        final double PERCENT_DEADBAND = 0.03;
        if (Math.abs(speeds.omegaRadiansPerSecond) < PERCENT_DEADBAND * getChassisMaxAngularVelocity()
            && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < PERCENT_DEADBAND * getChassisMaxLinearVelocityMetersPerSec())
            speeds = new ChassisSpeeds();

        runRawChassisSpeeds(ChassisSpeeds.discretize(speeds, 0.02));
    }

    /**
     * configure the PathPlanner {@link AutoBuilder} to make this robot follow the paths
     * */
    default void configureHolonomicPathPlannerAutoBuilder(Field2d field2d, BooleanSupplier shouldFlipPath) {
        configureHolonomicPathPlannerAutoBuilder(
                field2d, shouldFlipPath,
                new PIDConstants(5, 0 , 0),
                new PIDConstants(5, 0 , 0)
        );
    }

    /**
     * configure the PathPlanner {@link AutoBuilder} to make this robot follow the paths
     * */
    default void configureHolonomicPathPlannerAutoBuilder(Field2d field2d, BooleanSupplier shouldFlipPath, PIDConstants translationalPIDConstants, PIDConstants rotationalPIDConstants) {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getMeasuredChassisSpeedsRobotRelative,
                this::runRobotCentricChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        translationalPIDConstants, rotationalPIDConstants,
                        getChassisMaxLinearVelocityMetersPerSec(),
                        getChassisMaxLinearVelocityMetersPerSec() / getChassisMaxAngularVelocity(),
                        new ReplanningConfig(false, true)
                ),
                shouldFlipPath,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    final Pose2d[] trajectory = activePath.toArray(new Pose2d[0]);
                    Logger.recordOutput("Odometry/Trajectory", trajectory);
                    field2d.getObject("trajectory").setPoses(trajectory);
                }
        );
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );
    }

    /**
     * whether a chassis speed is small enough to be considered zero
     * @return true if the given chassis speeds is considered zero
     * */
    static boolean isZero(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.omegaRadiansPerSecond) < Math.toRadians(1)
                && Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.01
                && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.01;
    }

    /**
     *
     * */
    default ChassisSpeeds constrainAcceleration(
            ChassisSpeeds currentSpeeds, ChassisSpeeds desiredSpeeds,
            double linearSmoothOutTimeSeconds, double rotationSmoothOutTimeSeconds,
            double dtSecs) {
        final double
                MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = getChassisMaxLinearVelocityMetersPerSec()
                / linearSmoothOutTimeSeconds,
                MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = getChassisMaxAngularVelocity()
                / rotationSmoothOutTimeSeconds;

        Translation2d currentLinearVelocityMetersPerSec = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
                desiredLinearVelocityMetersPerSec = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond),
                linearVelocityDifference = desiredLinearVelocityMetersPerSec.minus(currentLinearVelocityMetersPerSec);

        final double maxLinearVelocityChangeIn1Period = MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ * dtSecs;
        final boolean desiredLinearVelocityReachableWithin1Period = linearVelocityDifference.getNorm() <= maxLinearVelocityChangeIn1Period;
        final Translation2d linearVelocityChangeVector = new Translation2d(maxLinearVelocityChangeIn1Period, linearVelocityDifference.getAngle()),
                newLinearVelocity = desiredLinearVelocityReachableWithin1Period ?
                desiredLinearVelocityMetersPerSec
                : currentLinearVelocityMetersPerSec.plus(linearVelocityChangeVector);

        final double angularVelocityDifference = desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond,
                maxAngularVelocityChangeIn1Period = MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * dtSecs,
                angularVelocityChange = Math.copySign(maxAngularVelocityChangeIn1Period, angularVelocityDifference);
        final boolean desiredAngularVelocityReachableWithin1Period = Math.abs(angularVelocityDifference) <= maxAngularVelocityChangeIn1Period;
        final double newAngularVelocity = desiredAngularVelocityReachableWithin1Period ?
                desiredSpeeds.omegaRadiansPerSecond
                : currentSpeeds.omegaRadiansPerSecond + angularVelocityChange;
        return new ChassisSpeeds(
                newLinearVelocity.getX(),
                newLinearVelocity.getY(),
                newAngularVelocity
        );
    }
}
