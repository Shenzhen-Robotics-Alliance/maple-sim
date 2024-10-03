package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import java.util.Arrays;
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
    private final GyroSimulation gyroSimulation;
    private final Translation2d[] moduleTranslations;
    private final SwerveDriveKinematics swerveDriveKinematics;

    /**
     * @param initialPoseOnField
     */
    public SwerveDriveSimulation(double robotMassWithBumpersKg, double bumperWidthMeters, double bumperLengthMeters, SwerveModuleSimulation[] moduleSimulations, Translation2d[] moduleTranslations,  GyroSimulation gyroSimulation, Pose2d initialPoseOnField) {
        super(
                new DriveTrainSimulationProfile(
                        moduleSimulations[0].getModuleTheoreticalSpeedMPS(),
                        moduleSimulations[0].getModuleMaxAccelerationMPSsq(robotMassWithBumpersKg, moduleSimulations.length),
                        moduleSimulations[0].getModuleTheoreticalSpeedMPS() / moduleTranslations[0].getNorm(),
                        moduleSimulations[0].getModuleMaxAccelerationMPSsq(robotMassWithBumpersKg, moduleSimulations.length) / moduleTranslations[0].getNorm(),
                        robotMassWithBumpersKg,
                        bumperWidthMeters,
                        bumperLengthMeters
                ).withAngularVelocityDamping(1).withLinearVelocityDamping(1),
                initialPoseOnField);

        this.moduleSimulations = moduleSimulations;
        this.moduleTranslations = moduleTranslations;
        this.swerveDriveKinematics = new SwerveDriveKinematics(moduleTranslations);
        this.gyroSimulation = gyroSimulation;
    }

    private double getGravityForceOnEachModule() {
        return profile.robotMass * 9.8 / moduleSimulations.length;
    }

    @Override
    public void simulationSubTick() {
        /* update the gyro simulation */
        gyroSimulation.updateSimulationSubTick(super.getAngularVelocity());

        /* simulate the translational friction force */
        simulateChassisFrictionForce();
        /* simulate the rotational friction torque */
        simulateChassisFrictionTorque();

        /* simulate the propelling force of each module */
        simulateModulePropellingForces();
    }

    private void simulateModulePropellingForces() {
        for (int i = 0; i < moduleSimulations.length; i++) {
            final Vector2 moduleWorldPosition = getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));
            final Vector2 moduleForce = moduleSimulations[i].updateSimulationSubTickGetModuleForce(
                    super.getLinearVelocity(moduleWorldPosition),
                    getSimulatedDriveTrainPose().getRotation(),
                    getGravityForceOnEachModule()
            );
            super.applyForce(
                    moduleForce,
                    moduleWorldPosition
            );
        }
    }

    private void simulateChassisFrictionForce() {
        final ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative = getModuleSpeeds()
                .minus(getDriveTrainSimulatedChassisSpeedsRobotRelative());
        final Translation2d floorAndModuleSpeedsDiffFieldRelative = new Translation2d(
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vxMetersPerSecond,
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vyMetersPerSecond
        ).rotateBy(getSimulatedDriveTrainPose().getRotation());

        SmartDashboard.putString("MapleArenaSimulation/difference", floorAndModuleSpeedsDiffFieldRelative.toString());

        final double FRICTION_FORCE_GAIN = 3.0,
                totalGrippingForce = moduleSimulations[0].getGrippingForceNewtons(getGravityForceOnEachModule())
                * moduleSimulations.length;
        final Vector2 frictionForce = Vector2.create(
                Math.min(FRICTION_FORCE_GAIN * totalGrippingForce * floorAndModuleSpeedsDiffFieldRelative.getNorm(), totalGrippingForce),
                floorAndModuleSpeedsDiffFieldRelative.getAngle().getRadians()
        );
        super.applyForce(frictionForce);
        SmartDashboard.putString("MapleArenaSimulation/chassisFrictionForce", frictionForce.toString());
    }

    private void simulateChassisFrictionTorque() {
        final double
                desiredRotationalMotionPercent = Math.abs(getDesiredSpeed().omegaRadiansPerSecond / getTheoreticalMaxAngularVelocity()),
                actualRotationalMotionPercent = Math.abs(getAngularVelocity() / getTheoreticalMaxAngularVelocity()),
                differenceBetweenFloorSpeedAndModuleSpeed = getModuleSpeeds().omegaRadiansPerSecond - getAngularVelocity(),
                grippingTorqueMagnitude =
                        moduleSimulations[0].getGrippingForceNewtons(getGravityForceOnEachModule())
                                * moduleTranslations[0].getNorm()
                                * moduleSimulations.length,
                FRICTION_TORQUE_GAIN = 1;

        if (actualRotationalMotionPercent < 0.01 && desiredRotationalMotionPercent < 0.02)
            super.setAngularVelocity(0);
        else super.applyTorque(Math.copySign(
                Math.min(FRICTION_TORQUE_GAIN * grippingTorqueMagnitude * Math.abs(differenceBetweenFloorSpeedAndModuleSpeed), grippingTorqueMagnitude),
                differenceBetweenFloorSpeedAndModuleSpeed
        ));
    }

    private ChassisSpeeds getDesiredSpeed() {
        return swerveDriveKinematics.toChassisSpeeds(
                Arrays.stream(moduleSimulations)
                        .map((SwerveModuleSimulation::getFreeSpinState))
                        .toArray(SwerveModuleState[]::new)
        );
    }

    private ChassisSpeeds getModuleSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(
                Arrays.stream(moduleSimulations)
                        .map((SwerveModuleSimulation::getCurrentState))
                        .toArray(SwerveModuleState[]::new)
        );
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

    public SwerveModuleSimulation[] getModules() {
        return moduleSimulations;
    }

    public static SwerveDriveSimulation createSwerve(
            double robotMassWidthBumpersKg,
            double trackLengthMeters, double trackWidthMeters,
            double bumperWidthMeters, double bumperLengthMeters,
            Supplier<SwerveModuleSimulation> swerveModuleFactory,
            GyroSimulation gyroSimulation,
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
                gyroSimulation,
                initialPoseOnField
        );
    }
}
