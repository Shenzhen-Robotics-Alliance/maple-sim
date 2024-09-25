package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import java.util.function.Supplier;

/**
 * <h1>Simulates a Swerve DriveTrain</h1>
 * <p>Made up of more than two {@link SwerveModuleSimulation}</p>
 * <h3>To Simulate Odometry: </h3>
 * <p>1. Retrieve the encoder readings from {@link SwerveModuleSimulation}.</p>
 * <p>2. Use a {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry} to estiamte the pose of your robot.
 * <a href="https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html">250HZ Odometry</a> IS SUPPORTED</p>
 * <p>3. Optionally, obtain real robot pose from {@link AbstractDriveTrainSimulation}.getSimulatedDriveTrainPose() and feed it to <a href="https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html#updating-the-simulation-world">photon vision simulation</a> to simulate vision</p>
 * <p>this way, you get a realistic simulation of the odometry, which accounts for measurement errors due to skidding and imu drifting.</p>
 * */
public class SwerveDriveSimulation extends AbstractDriveTrainSimulation {
    private final SwerveModuleSimulation[] moduleSimulations;
    private final Translation2d[] moduleTranslations;
    private final SwerveDriveKinematics swerveDriveKinematics;

    /**
     * @param initialPoseOnField
     */
    public SwerveDriveSimulation(double robotMassWithBumpersKg, double bumperWidthMeters, double bumperLengthMeters, SwerveModuleSimulation[] moduleSimulations, Translation2d[] moduleTranslations, Pose2d initialPoseOnField) {
        super(
                new DriveTrainSimulationProfile(
                        moduleSimulations[0].getModuleTheoreticalSpeedMPS(),
                        moduleSimulations[0].getModuleMaxAccelerationMPSsq(robotMassWithBumpersKg, moduleSimulations.length),
                        moduleSimulations[0].getModuleTheoreticalSpeedMPS() / moduleTranslations[0].getNorm(),
                        moduleSimulations[0].getModuleMaxAccelerationMPSsq(robotMassWithBumpersKg, moduleSimulations.length) / moduleTranslations[0].getNorm(),
                        robotMassWithBumpersKg,
                        bumperWidthMeters,
                        bumperLengthMeters
                ).withAngularVelocityDamping(0.3).withLinearVelocityDamping(0.3),
                initialPoseOnField);

        this.moduleSimulations = moduleSimulations;
        this.moduleTranslations = moduleTranslations;
        this.swerveDriveKinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    @Override
    public void simulationSubTick() {
        if (DriverStation.isDisabled()) {
            simulateChassisAngularFriction();
            simulateChassisLinearFriction();
            return;
        }

        final SwerveModuleState[] actualModuleStates = swerveDriveKinematics.toSwerveModuleStates(
                getDriveTrainSimulatedChassisSpeedsRobotRelative()
        );
        for (int i = 0; i < moduleSimulations.length; i++) {
            final Vector2 moduleCurrentGroundVelocityWorldRelative = Vector2.create(
                    actualModuleStates[i].speedMetersPerSecond,
                    actualModuleStates[i].angle.getRadians()
            ).rotate(getSimulatedDriveTrainPose().getRotation().getRadians());
            final Vector2 moduleForce = moduleSimulations[i].updateSimulationSubTickGetModuleForce(
                    moduleCurrentGroundVelocityWorldRelative,
                    getSimulatedDriveTrainPose().getRotation(),
                    profile.robotMass * 9.8 / moduleSimulations.length
            );
            final Vector2 moduleWorldPosition = getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));

            super.applyForce(
                    moduleForce,
                    moduleWorldPosition
            );
        }
    }

    public double getTheoreticalMaxLinearVelocity() {
        return moduleSimulations[0].getModuleTheoreticalSpeedMPS();
    }

    public double getTheoreticalMaxLinearAcceleration() {
        return moduleSimulations[0].getModuleMaxAccelerationMPSsq(profile.robotMass, moduleSimulations.length);
    }

    public double getTheoreticalMaxAngularVelocity() {
        return moduleSimulations[0].getModuleTheoreticalSpeedMPS() / moduleTranslations[0].getNorm();
    }

    public double getSwerveDriveMaxAngularAcceleration() {
        return moduleSimulations[0].getTheoreticalPropellingForcePerModule(profile.robotMass, moduleSimulations.length)
                * moduleTranslations[0].getNorm() * moduleSimulations.length / super.getMass().getInertia();
    }

    public static SwerveDriveSimulation createSwerve(
            double robotMassWidthBumpersKg,
            double trackLengthMeters, double trackWidthMeters,
            double bumperWidthMeters, double bumperLengthMeters,
            Supplier<SwerveModuleSimulation> swerveModuleFactory,
            Pose2d initialPoseOnField) {
        return new SwerveDriveSimulation(
                robotMassWidthBumpersKg,
                bumperWidthMeters,
                bumperLengthMeters,
                new SwerveModuleSimulation[]{
                        swerveModuleFactory.get(), swerveModuleFactory.get(),
                        swerveModuleFactory.get(), swerveModuleFactory.get()
                },
                new Translation2d[]{
                        new Translation2d(trackLengthMeters / 2, trackWidthMeters / 2),
                        new Translation2d(trackLengthMeters / 2, -trackWidthMeters / 2),
                        new Translation2d(-trackLengthMeters / 2, trackWidthMeters / 2),
                        new Translation2d(-trackLengthMeters / 2, -trackWidthMeters / 2)
                },
                initialPoseOnField
        );
    }
}
