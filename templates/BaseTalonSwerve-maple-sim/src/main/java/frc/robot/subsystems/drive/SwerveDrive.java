package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The {@code Drive} interface represents a subsystem responsible for controlling the drive system of a robot. This
 * includes methods for controlling the movement, managing swerve drive modules, tracking the robot's position and
 * orientation, and other related tasks.
 */
public interface SwerveDrive extends Subsystem {

    /**
     * Controls the robot's movement based on the provided translation and rotation values.
     *
     * @param translation The translation of the robot, represented as a 2D vector with x and y components.
     * @param rotation The rotation of the robot around its center.
     * @param fieldRelative Whether the movement is field-relative or robot-relative.
     * @param isOpenLoop Whether the drive should use open-loop control (without feedback).
     */
    void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop);

    /**
     * Sets the desired states for each swerve module in the drive system.
     *
     * @param desiredStates The desired states for all swerve modules, including speed and angle information.
     */
    void setModuleStates(SwerveModuleState[] desiredStates);

    /**
     * Retrieves the current states of all swerve modules in the drive system.
     *
     * @return An array of {@link SwerveModuleState} objects representing the current state (speed and angle) of each
     *     swerve module. The array will contain one element per swerve module (typically 4 for a standard swerve drive
     *     system).
     */
    SwerveModulePosition[] getModulePositions();

    /**
     * Retrieves the current states of all swerve modules in the drive system.
     *
     * @return An array of {@link SwerveModuleState} objects representing the current state (speed and angle) of each
     *     swerve module.
     */
    SwerveModuleState[] getModuleStates();

    /**
     * Retrieves the current yaw angle from the robot's gyro, note that this might not reflect the actual rotation of a
     * robot
     *
     * @return A {@link Rotation2d} object representing the current yaw angle of the robot. The angle is measured in
     *     degrees and typically ranges from -180° to +180°.
     */
    Rotation2d getGyroYaw();

    /**
     * Gets the robot's current position and orientation in the field.
     *
     * @return The current pose of the robot as a {@link Pose2d} object.
     */
    Pose2d getPose();

    /**
     * Sets the robot's position and orientation to a specific pose.
     *
     * @param pose The desired pose to set the robot's position and orientation to.
     */
    void setPose(Pose2d pose);

    /**
     * Gets the robot's current heading (orientation) in the field.
     *
     * @return The current heading of the robot as a {@link Rotation2d} object.
     */
    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /** sets the heading of the robot */
    default void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    /** Resets the robot's heading to zero, effectively "zeroing" its orientation. */
    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);

    void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);

    default void initSwerveWidget(String key) {
        SmartDashboard.putData(key, (builder) -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Front Right Angle", () -> getModuleStates()[1].angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Left Angle", () -> getModuleStates()[2].angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Right Angle", () -> getModuleStates()[3].angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
        });
    }
}
