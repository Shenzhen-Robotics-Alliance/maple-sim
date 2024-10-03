package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import java.util.Arrays;
import java.util.function.Supplier;

/**
 * <h1>Simulates a Swerve Drivetrain.</h1>
 *
 * <p>This class simulates a swerve drivetrain made up of more than two {@link SwerveModuleSimulation} objects.</p>
 *
 * <h3>To Simulate Odometry:</h3>
 * <ul>
 *   <li>Retrieve the encoder readings from {@link SwerveModuleSimulation}.</li>
 *   <li>Use {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry} to estimate the pose of your robot.</li>
 *   <li><a href="https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html">250Hz Odometry</a> is supported.</li>
 *   <li>Optionally, obtain the real robot pose from {@link AbstractDriveTrainSimulation#getSimulatedDriveTrainPose()} and feed it to the
 *   <a href="https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html#updating-the-simulation-world">PhotonVision simulation</a> to simulate vision.</li>
 * </ul>
 * <p>This approach provides a realistic simulation of odometry, accounting for measurement errors due to skidding and IMU drifting.</p>
 * */
public class SwerveDriveSimulation extends AbstractDriveTrainSimulation {
    private final SwerveModuleSimulation[] moduleSimulations;
    private final GyroSimulation gyroSimulation;
    private final Translation2d[] moduleTranslations;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final double gravityForceOnEachModule;

    /**
     * <h2>Creates a Swerve Drive Simulation.</h2>
     *
     * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper dimensions, module simulations, module translations, gyro simulation, and initial pose on the field.</p>
     *
     * @param robotMassWithBumpersKg the total mass of the robot including bumpers, in kilograms
     * @param bumperWidthMeters the width of the robot's bumpers, in meters
     * @param bumperLengthMeters the length of the robot's bumpers, in meters
     * @param moduleSimulations an array of {@link SwerveModuleSimulation} objects representing the modules of the drivetrain
     * @param moduleTranslations an array of {@link Translation2d} objects representing the position of each module relative to the robot center
     * @param gyroSimulation a {@link GyroSimulation} object used to simulate the IMU for odometry
     * @param initialPoseOnField the initial pose of the drivetrain in the simulation world, represented as a {@link Pose2d}
     * */
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

        this.gravityForceOnEachModule = profile.robotMass * 9.8 / moduleSimulations.length;
    }

    /**
     * <h2>Constructs a Swerve Drive Simulation.</h2>
     *
     * <p>This constructor initializes a swerve drive simulation with the given robot mass, track dimensions, bumper dimensions, module factory, gyro simulation, and initial pose on the field. It calls the main constructor internally.</p>
     *
     * @param robotMassWidthBumpersKg the total mass of the robot including bumpers, in kilograms
     * @param trackLengthMeters the length of the robot's track, in meters
     * @param trackWidthMeters the width of the robot's track, in meters
     * @param bumperWidthMeters the width of the robot's bumpers, in meters
     * @param bumperLengthMeters the length of the robot's bumpers, in meters
     * @param swerveModuleFactory a supplier that provides instances of {@link SwerveModuleSimulation} for each swerve module
     * @param gyroSimulation the {@link GyroSimulation} instance to simulate the robot's IMU
     * @param initialPoseOnField the initial pose of the robot in the simulation world, represented as a {@link Pose2d}
     */
    public SwerveDriveSimulation(
            double robotMassWidthBumpersKg,
            double trackLengthMeters, double trackWidthMeters,
            double bumperWidthMeters, double bumperLengthMeters,
            Supplier<SwerveModuleSimulation> swerveModuleFactory,
            GyroSimulation gyroSimulation,
            Pose2d initialPoseOnField) {

        // Use this() to call the main constructor with calculated parameters
        this(
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

    /**
     * <h2>Updates the Swerve Drive Simulation.</h2>
     *
     * <p>This method performs the following actions during each sub-tick of the simulation:</p>
     * <ul>
     *   <li>Applies the translational friction force to the physics engine.</li>
     *   <li>Applies the rotational friction torque to the physics engine.</li>
     *   <li>Updates the simulation of each swerve module.</li>
     *   <li>Applies the propelling forces of the modules to the physics engine.</li>
     *   <li>Updates the gyro simulation of the drivetrain.</li>
     * </ul>
     * */
    @Override
    public void simulationSubTick() {
        simulateChassisFrictionForce();

        simulateChassisFrictionTorque();

        simulateModulePropellingForces();

        gyroSimulation.updateSimulationSubTick(super.getAngularVelocity());
    }

    private Translation2d previousModuleSpeedsFieldRelative = new Translation2d();

    /**
     * <h2>Simulates the Translational Friction Force and Applies It to the Physics Engine.</h2>
     *
     * <p>This method simulates the translational friction forces acting on the robot and applies them to the physics engine. There are two components of the friction forces:</p>
     * <ul>
     *   <li>A portion of the friction force pushes the robot from its current ground speeds ({@link #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current module speeds ({@link #getModuleSpeeds()}).</li>
     *   <li>Another portion of the friction force is the centripetal force, which occurs when the chassis changes its direction of movement.</li>
     * </ul>
     *
     * <p>The total friction force should not exceed the tire's grip limit.</p>
     * */
    private void simulateChassisFrictionForce() {
        final ChassisSpeeds moduleSpeeds = getModuleSpeeds();

        /* The friction force that tries to bring the chassis from floor speeds to module speeds */
        final ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative = moduleSpeeds
                .minus(getDriveTrainSimulatedChassisSpeedsRobotRelative());
        final Translation2d floorAndModuleSpeedsDiffFieldRelative = new Translation2d(
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vxMetersPerSecond,
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vyMetersPerSecond
        ).rotateBy(getSimulatedDriveTrainPose().getRotation());
        final double FRICTION_FORCE_GAIN = 3.0,
                totalGrippingForce = moduleSimulations[0].getGrippingForceNewtons(gravityForceOnEachModule)
                        * moduleSimulations.length;
        final Vector2 speedsDifferenceFrictionForce = Vector2.create(
                Math.min(FRICTION_FORCE_GAIN * totalGrippingForce * floorAndModuleSpeedsDiffFieldRelative.getNorm(), totalGrippingForce),
                floorAndModuleSpeedsDiffFieldRelative.getAngle().getRadians()
        );

        /* the centripetal friction force during turning */
        final ChassisSpeeds moduleSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                moduleSpeeds,
                getSimulatedDriveTrainPose().getRotation()
        );
        final Rotation2d dTheta = GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative)
                .getAngle()
                .minus(previousModuleSpeedsFieldRelative.getAngle());
        final double orbitalAngularVelocity = dTheta.getRadians() / SimulatedArena.getSimulationDt();
        final Rotation2d centripetalForceDirection = previousModuleSpeedsFieldRelative.getAngle()
                .plus(Rotation2d.fromDegrees(90));
        final Vector2 centripetalFrictionForce = Vector2.create(
                previousModuleSpeedsFieldRelative.getNorm() * orbitalAngularVelocity * profile.robotMass,
                centripetalForceDirection.getRadians()
        );
        previousModuleSpeedsFieldRelative = GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative);

        /* apply force to physics engine */
        final Vector2 totalFrictionForceUnlimited = centripetalFrictionForce.copy()
                .add(speedsDifferenceFrictionForce),
                totalFrictionForce = Vector2.create(
                        Math.min(totalGrippingForce, totalFrictionForceUnlimited.getMagnitude()),
                        totalFrictionForceUnlimited.getDirection()
                );
        super.applyForce(totalFrictionForce);
    }

    /**
     * <h2>Simulates the Rotational Friction Torque and Applies It to the Physics Engine.</h2>
     *
     * <p>This method simulates the rotational friction torque acting on the robot and applies them to the physics engine.</p>
     * <p>The friction torque pushes the robot from its current ground angular velocity ({@link #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current modules' angular velocity ({@link #getModuleSpeeds()}).</p>
     * */
    private void simulateChassisFrictionTorque() {
        final double
                desiredRotationalMotionPercent = Math.abs(getDesiredSpeed().omegaRadiansPerSecond / getTheoreticalMaxAngularVelocity()),
                actualRotationalMotionPercent = Math.abs(getAngularVelocity() / getTheoreticalMaxAngularVelocity()),
                differenceBetweenFloorSpeedAndModuleSpeed = getModuleSpeeds().omegaRadiansPerSecond - getAngularVelocity(),
                grippingTorqueMagnitude =
                        moduleSimulations[0].getGrippingForceNewtons(gravityForceOnEachModule)
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

    /**
     * <h2>Simulates the Translational Friction Force and Applies It to the Physics Engine.</h2>
     *
     * <p>This method simulates the translational friction forces acting on the robot and applies them to the physics engine. There are two components of the friction forces:</p>
     * <ul>
     *   <li>A portion of the friction force pushes the robot from its current ground speeds ({@link #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current module speeds ({@link #getModuleSpeeds()}).</li>
     *   <li>Another portion of the friction force is the centripetal force, which occurs when the chassis changes its direction of movement.</li>
     * </ul>
     *
     * <p>The total friction force should not exceed the tire's grip limit.</p>
     * */
    private void simulateModulePropellingForces() {
        for (int i = 0; i < moduleSimulations.length; i++) {
            final Vector2 moduleWorldPosition = getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));
            final Vector2 moduleForce = moduleSimulations[i].updateSimulationSubTickGetModuleForce(
                    super.getLinearVelocity(moduleWorldPosition),
                    getSimulatedDriveTrainPose().getRotation(),
                    gravityForceOnEachModule
            );
            super.applyForce(
                    moduleForce,
                    moduleWorldPosition
            );
        }
    }

    /**
     * <h2>Obtains the Chassis Speeds the Modules Are Attempting to Achieve.</h2>
     *
     * <p>This method returns the desired chassis speeds that the modules are trying to reach. If the robot maintains the current driving voltage and steering position for a long enough period, it will achieve these speeds.</p>
     *
     * @return the desired chassis speeds, robot-relative
     */
    private ChassisSpeeds getDesiredSpeed() {
        return swerveDriveKinematics.toChassisSpeeds(
                Arrays.stream(moduleSimulations)
                        .map((SwerveModuleSimulation::getFreeSpinState))
                        .toArray(SwerveModuleState[]::new)
        );
    }

    /**
     * <h2>Obtains the Current Chassis Speeds of the Modules.</h2>
     *
     * <p>This method estimates the chassis speeds of the robot based on the swerve states of the modules.</p>
     * <p><strong>Note:</strong> These speeds might not represent the actual floor speeds due to potential skidding.</p>
     *
     * @return the module speeds, robot-relative
     */
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
}
