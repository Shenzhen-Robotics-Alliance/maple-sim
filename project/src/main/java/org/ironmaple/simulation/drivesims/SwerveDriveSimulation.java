package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimRobot;
import org.ironmaple.simulation.SimulationArena.SimulationTiming;
import org.ironmaple.simulation.drivesims.configs.SwerveConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 *
 *
 * <h1>Simulates a Swerve Drivetrain.</h1>
 *
 * <h3>1. Purpose</h3>
 *
 * <p>This class simulates a swerve drivetrain composed of more than two {@link SwerveModuleSimulation} modules.
 *
 * <p>It provides a realistic modeling of drivetrain physics, replicating wheel grip and motor propulsion for an actual
 * swerve drive.
 *
 * <h3>2. Simulation Dynamics</h3>
 *
 * <ul>
 *   <li>1. Propelling forces generated by the drive motors, computed by
 *       {@link SwerveModuleSimulation#updateSimulationSubTickGetModuleForce(Vector2, Rotation2d, double)}.
 *   <li>2. Friction forces generated by the wheels that "pull" the robot from its current ground velocity to the module
 *       velocities, both translational and rotational.
 *   <li>3. Centripetal forces generated by the steering when the drivetrain makes a turn.
 * </ul>
 *
 * <h3>3. Odometry Simulation</h3>
 *
 * <p>To simulate odometry, follow these steps:
 *
 * <ul>
 *   <li>Obtain the {@link SwerveModuleSimulation} instances through {@link #getModules()}.
 *   <li>Create an <a href='https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md'>IO
 *       Implementation</a> that wraps around {@link SwerveModuleSimulation} to retrieve encoder readings.
 *   <li>Update a {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} using the encoder readings, similar to
 *       how you would on a real robot.
 * </ul>
 *
 * <p>Refer to the <a
 * href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java'>ModuleIOSim.java</a>
 * example project for more details.
 *
 * <h3>Vision Simulation</h3>
 *
 * <p>You can obtain the real robot pose from {@link #getSimulatedPose()} and feed it to the <a
 * href="https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html#updating-the-simulation-world">PhotonVision
 * simulation</a> to simulate vision.
 */
public class SwerveDriveSimulation extends DriveTrainSimulation {
    protected final SimulationTiming timing;
    protected final SwerveModuleSimulation[] moduleSimulations;
    protected final GyroSimulation gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;
    private final double gravityForceOnEachModule;
    private final SwerveConfig config;

    /**
     *
     *
     * <h2>Creates a Swerve Drive Simulation.</h2>
     *
     * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper dimensions, module
     * simulations, module translations, gyro simulation, and initial pose on the field.
     *
     * @param config a {@link SwerveConfig} instance containing the configurations of * this drivetrain
     * @param initialPoseOnField the initial pose of the drivetrain in the simulation world, represented as a
     *     {@link Pose2d}
     */
    public SwerveDriveSimulation(SimRobot robot, SwerveConfig config, Pose2d initialPoseOnField) {
        super(config, initialPoseOnField);
        this.timing = robot.timing();
        this.config = config;
        this.moduleTranslations = config.moduleTranslations;
        this.moduleSimulations = Arrays.stream(config.moduleTranslations)
                .map(translation2d -> new SwerveModuleSimulation(robot, config.swerveModuleConfig))
                .toArray(SwerveModuleSimulation[]::new);
        this.gyroSimulation = new GyroSimulation(timing, config.gyroConfig);

        super.setLinearDamping(1.4);
        super.setAngularDamping(1.4);
        this.kinematics = new SwerveDriveKinematics(moduleTranslations);

        this.gravityForceOnEachModule = (config.robotMassKg * 9.8) / moduleSimulations.length;

        RuntimeLog.debug("created swerve drive simulation");
    }

    /**
     *
     *
     * <h2>Updates the Swerve Drive Simulation.</h2>
     *
     * <p>This method performs the following actions during each sub-tick of the simulation:
     *
     * <ul>
     *   <li>Applies the translational friction force to the physics engine.
     *   <li>Applies the rotational friction torque to the physics engine.
     *   <li>Updates the simulation of each swerve module.
     *   <li>Applies the propelling forces of the modules to the physics engine.
     *   <li>Updates the gyro simulation of the drivetrain.
     * </ul>
     */
    @Override
    public void simulationSubTick() {
        simulateChassisFrictionForce();

        simulateChassisFrictionTorque();

        simulateModulePropellingForces();

        gyroSimulation.updateSimulationSubTick(super.getAngularVelocity());
    }

    private Translation2d previousModuleSpeedsFieldRelative = new Translation2d();

    /**
     *
     *
     * <h2>Simulates the Translational Friction Force and Applies It to the Physics Engine.</h2>
     *
     * <p>This method simulates the translational friction forces acting on the robot and applies them to the physics
     * engine. There are two components of the friction forces:
     *
     * <ul>
     *   <li>A portion of the friction force pushes the robot from its current ground speeds
     *       ({@link #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current module speeds
     *       ({@link #getModuleSpeeds()}).
     *   <li>Another portion of the friction force is the centripetal force, which occurs when the chassis changes its
     *       direction of movement.
     * </ul>
     *
     * <p>The total friction force should not exceed the tire's grip limit.
     */
    private void simulateChassisFrictionForce() {
        final ChassisSpeeds moduleSpeeds = getModuleSpeeds();

        /* The friction force that tries to bring the chassis from floor speeds to module speeds */
        final ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative = moduleSpeeds
                .minus(getDriveTrainSimulatedChassisSpeedsRobotRelative());
        final Translation2d floorAndModuleSpeedsDiffFieldRelative = new Translation2d(
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vxMetersPerSecond,
                differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative.vyMetersPerSecond)
                .rotateBy(getSimulatedPose().getRotation());
        final double FRICTION_FORCE_GAIN = 3.0,
                totalGrippingForce = moduleSimulations[0].getGrippingForceNewtons(gravityForceOnEachModule)
                        * moduleSimulations.length;
        final Vector2 speedsDifferenceFrictionForce = Vector2.create(
                Math.min(
                        FRICTION_FORCE_GAIN * totalGrippingForce * floorAndModuleSpeedsDiffFieldRelative.getNorm(),
                        totalGrippingForce),
                MapleCommonMath.getAngle(floorAndModuleSpeedsDiffFieldRelative).getRadians());

        /* the centripetal friction force during turning */
        final ChassisSpeeds moduleSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                moduleSpeeds, getSimulatedPose().getRotation());
        final Rotation2d dTheta = MapleCommonMath.getAngle(
                GeometryConvertor.getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative))
                .minus(MapleCommonMath.getAngle(previousModuleSpeedsFieldRelative));

        final double orbitalAngularVelocity = dTheta.getRadians() / timing.dt;
        final Rotation2d centripetalForceDirection = MapleCommonMath.getAngle(previousModuleSpeedsFieldRelative)
                .plus(Rotation2d.fromDegrees(90));
        final Vector2 centripetalFrictionForce = Vector2.create(
                previousModuleSpeedsFieldRelative.getNorm() * orbitalAngularVelocity * config.robotMassKg,
                centripetalForceDirection.getRadians());
        previousModuleSpeedsFieldRelative = GeometryConvertor
                .getChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative);

        /* apply force to physics engine */
        final Vector2 totalFrictionForceUnlimited = centripetalFrictionForce.copy().add(speedsDifferenceFrictionForce),
                totalFrictionForce = Vector2.create(
                        Math.min(totalGrippingForce, totalFrictionForceUnlimited.getMagnitude()),
                        totalFrictionForceUnlimited.getDirection());
        super.applyForce(totalFrictionForce);
    }

    /**
     *
     *
     * <h2>Simulates the Rotational Friction Torque and Applies It to the Physics Engine.</h2>
     *
     * <p>This method simulates the rotational friction torque acting on the robot and applies them to the physics
     * engine.
     *
     * <p>The friction torque pushes the robot from its current ground angular velocity
     * ({@link #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current modules' angular velocity
     * ({@link #getModuleSpeeds()}).
     */
    private void simulateChassisFrictionTorque() {
        final double desiredRotationalMotionPercent = Math
                .abs(getDesiredSpeed().omegaRadiansPerSecond / getTheoreticalMaxAngularVelocity()),
                actualRotationalMotionPercent = Math.abs(getAngularVelocity() / getTheoreticalMaxAngularVelocity()),
                differenceBetweenFloorSpeedAndModuleSpeed = getModuleSpeeds().omegaRadiansPerSecond
                        - getAngularVelocity(),
                grippingTorqueMagnitude = moduleSimulations[0].getGrippingForceNewtons(gravityForceOnEachModule)
                        * moduleTranslations[0].getNorm()
                        * moduleSimulations.length,
                FRICTION_TORQUE_GAIN = 1;

        if (actualRotationalMotionPercent < 0.01 && desiredRotationalMotionPercent < 0.02)
            super.setAngularVelocity(0);
        else
            super.applyTorque(Math.copySign(
                    Math.min(
                            FRICTION_TORQUE_GAIN
                                    * grippingTorqueMagnitude
                                    * Math.abs(differenceBetweenFloorSpeedAndModuleSpeed),
                            grippingTorqueMagnitude),
                    differenceBetweenFloorSpeedAndModuleSpeed));
    }

    /**
     *
     *
     * <h2>Simulates the Translational Friction Force and Applies It to the Physics Engine.</h2>
     *
     * <p>This method simulates the translational friction forces acting on the robot and applies them to the physics
     * engine. There are two components of the friction forces:
     *
     * <ul>
     *   <li>A portion of the friction force pushes the robot from its current ground speeds
     *       ({@link #getDriveTrainSimulatedChassisSpeedsRobotRelative()}) toward its current module speeds
     *       ({@link #getModuleSpeeds()}).
     *   <li>Another portion of the friction force is the centripetal force, which occurs when the chassis changes its
     *       direction of movement.
     * </ul>
     *
     * <p>The total friction force should not exceed the tire's grip limit.
     */
    private void simulateModulePropellingForces() {
        for (int i = 0; i < moduleSimulations.length; i++) {
            final Vector2 moduleWorldPosition = getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));
            final Vector2 moduleForce = moduleSimulations[i].updateSimulationSubTickGetModuleForce(
                    super.getLinearVelocity(moduleWorldPosition),
                    getSimulatedPose().getRotation(),
                    gravityForceOnEachModule);
            super.applyForce(moduleForce, moduleWorldPosition);
        }
    }

    /**
     *
     *
     * <h2>Obtains the Chassis Speeds the Modules Are Attempting to Achieve.</h2>
     *
     * <p>This method returns the desired chassis speeds that the modules are trying to reach. If the robot maintains
     * the current driving voltage and steering position for a long enough period, it will achieve these speeds.
     *
     * @return the desired chassis speeds, robot-relative
     */
    private ChassisSpeeds getDesiredSpeed() {
        return kinematics.toChassisSpeeds(Arrays.stream(moduleSimulations)
                .map((SwerveModuleSimulation::getFreeSpinState))
                .toArray(SwerveModuleState[]::new));
    }

    /**
     *
     *
     * <h2>Obtains the Current Chassis Speeds of the Modules.</h2>
     *
     * <p>This method estimates the chassis speeds of the robot based on the swerve states of the modules.
     *
     * <p><strong>Note:</strong> These speeds might not represent the actual floor speeds due to potential skidding.
     *
     * @return the module speeds, robot-relative
     */
    private ChassisSpeeds getModuleSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(moduleSimulations)
                .map((SwerveModuleSimulation::getCurrentState))
                .toArray(SwerveModuleState[]::new));
    }

    public double getTheoreticalMaxLinearVelocity() {
        return moduleSimulations[0].getModuleTheoreticalSpeedMPS();
    }

    public double getTheoreticalMaxLinearAcceleration() {
        return moduleSimulations[0].getModuleMaxAccelerationMPSsq(config.robotMassKg, moduleSimulations.length);
    }

    public double getTheoreticalMaxAngularVelocity() {
        return moduleSimulations[0].getModuleTheoreticalSpeedMPS() / moduleTranslations[0].getNorm();
    }

    public double getSwerveDriveMaxAngularAcceleration() {
        return moduleSimulations[0].getTheoreticalPropellingForcePerModule(config.robotMassKg, moduleSimulations.length)
                * moduleTranslations[0].getNorm()
                * moduleSimulations.length
                / super.getMass().getInertia();
    }

    public SwerveModuleSimulation[] getModules() {
        return moduleSimulations;
    }

    public GyroSimulation getGyroSimulation() {
        return this.gyroSimulation;
    }
}
