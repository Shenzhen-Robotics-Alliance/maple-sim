package org.ironmaple.simulation.opponentsim;

import static edu.wpi.first.units.Units.*;
// TODO

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.*;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.opponentsim.pathfinding.MapleADStar;
import org.ironmaple.utils.FieldMirroringUtils;

public abstract class SmartOpponent extends SubsystemBase {
    /// Publishers
    protected StringPublisher statePublisher;
    protected StructPublisher<Pose2d> posePublisher;
    // String used in telemetry for alliance.
    protected String allianceString;
    /// The SmartOpponentConfig to use.
    protected SmartOpponentConfig config;
    // The opponent manager if set
    protected OpponentManager manager;
    /// The drivetrain simulation.
    protected SelfControlledSwerveDriveSimulation drivetrainSim;
    /// The Manipulator Sim
    protected OpponentManipulatorSim manipulatorSim;
    /// The pathplanner config
    protected RobotConfig pathplannerConfig;
    /// Pathplanner HolonomicDriveController
    protected PPHolonomicDriveController driveController;
    // Pathfinding class cloned for modification.
    protected final MapleADStar mapleADStar;
    // Behavior Chooser Publisher
    protected StringPublisher selectedBehaviorPublisher;
    // Target Pose
    protected Pair<String, Pose2d> target;
    // Latest updated obstacle list
    protected final List<Pair<Translation2d, Translation2d>> latestObstacles = new ArrayList<>();
    // Obstacles are dynamically updated by the method that requires it.
    protected static double lastObstaclePoll = 0;

    /**
     * The SmartOpponent base abstracted class.
     *
     * @param config the {@link SmartOpponentConfig} to base the opponent off of.
     */
    public SmartOpponent(SmartOpponentConfig config) {
        /// Create and verify config.
        this.config = config;
        config.validateConfigs(); // Throw an error if the config is invalid.
        if (config.manager != null) {
            this.manager = config.manager;
        }
        this.driveController = new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5));
        // Cloned Pathfinder for use here.
        this.mapleADStar = new MapleADStar();
        // Preset an empty manipulator
        this.manipulatorSim = new OpponentManipulatorSim(this);
        /// Initialize simulations
        this.drivetrainSim = config.chassis.createDriveTrainSim(config.queeningPose);
        this.pathplannerConfig = config.chassis.updatePathplannerConfig();
        // Alliance string for telemetry.
        this.allianceString = DriverStation.Alliance.Blue.equals(config.alliance) ? "Blue Alliance/" : "Red Alliance/";
        // NetworkTable setup.
        this.statePublisher = NetworkTableInstance.getDefault()
                .getStringTopic(config.telemetryPath + "SimulatedOpponents/States/" + allianceString + config.name
                        + "'s Current State")
                .publish();
        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(
                        config.telemetryPath + "SimulatedOpponents/Poses/" + allianceString + config.name + "'s Pose2d",
                        Pose2d.struct)
                .publish();
        this.selectedBehaviorPublisher = NetworkTableInstance.getDefault()
                .getTable(config.telemetryPath + "SimulatedOpponents/Behaviors/" + allianceString + config.name
                        + "'s Behaviors")
                .getStringTopic("selected")
                .publish();
        /// Adds the required states to run the {@link org.ironmaple.simulation.opponentsim.SmartOpponent}.
        config.withState("Standby", this::standbyState);
        config.withState("Starting", () -> startingState("Collect"));
        config.withState("Collect", this::collectState);
        config.withState("Score", this::scoreState);
        setState("Standby");
        /// Adds options to the behavior sendable chooser.
        config.withBehavior("Disabled", runState("Standby", true), true);
        config.withBehavior("Enabled", runState("Starting", true));
        // Update the chooser and then publish it.
        SmartDashboard.putData(
                config.smartDashboardPath + "SimulatedOpponents/Behaviors/" + allianceString + config.name
                        + "'s Behaviors",
                config.updateBehaviorChooser());
        /// Run behavior command when changed.
        config.getBehaviorChooser().onChange(Command::schedule);
        /// Finally, add our simulation
        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSim.getDriveTrainSimulation());
        if (config.isAutoEnable) {
            RobotModeTriggers.teleop()
                    .onTrue(Commands.runOnce(
                            () -> config.getBehaviorChooser().getSelected().schedule()));
            RobotModeTriggers.disabled().onTrue(standbyState());
        }
        /// If a manager is set register with it
        if (config.manager != null) {
            config.manager.registerOpponent(this);
        }
        // Caches weighted poses for a faster search later.
        config.loadWeightedPoses();
    }

    /**
     * Updates the selected behavior {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command>}, as if it was
     * changed from the dashboard. This is done with a {@link StringPublisher} pointing to the
     * {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} selected string.
     *
     * @param behavior which option to select.
     * @return a command that updated the selected behavior.
     */
    protected Command setSelectedBehavior(String behavior) {
        return runOnce(() -> selectedBehaviorPublisher.set(behavior));
    }

    /**
     * The standby state to run.
     *
     * @return a Command that runs the state.
     */
    protected Command standbyState() {
        return runOnce(() -> drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false))
                .andThen(runOnce(() -> drivetrainSim.setSimulationWorldPose(config.queeningPose)))
                .ignoringDisable(true);
    }

    /**
     * The starting state to run.
     *
     * @return a Command that runs the state.
     */
    protected Command startingState(String nextState) {
        return runOnce(() -> drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false))
                .andThen(runOnce(() -> drivetrainSim.setSimulationWorldPose(config.initialPose)))
                .andThen(Commands.waitSeconds(0.25))
                .finallyDo(() -> setState(nextState))
                .ignoringDisable(config.isAutoEnable); // Only move to the field while disabled if autoEnable is off.
    }

    /**
     * The collect state to run.
     *
     * <p>
     *
     * <h1>Commands should be structured like below to prevent InstantCommands, causing the opponent to cycle states
     * without the command finishing.</h1>
     *
     * <p>
     *
     * <p>
     *
     * <h1>Good </h1>
     *
     * <pre><code>
     *     pathfind(getRandomFromMap(config.getPoseMap).withTimeout(7)) /// Go to element
     *     .andThen(run(() -> Runnable).withTimeout(0.5)) /// Run intake
     *     .andThen(runOnce(() -> Runnable)) /// Stop Intake
     *     .andThen(Commands.waitSeconds(0.5)) /// Wait before continuing
     *     .finallyDo(() -> setState("Score")); /// Now cycle to next state.
     * </code></pre>
     *
     * <p>
     *
     * <p>
     *
     * <h1>Bad </h1>
     *
     * <pre><code>
     *     pathfind
     *     /// This adds the command then adds the timeout to the entire command.
     *     .andThen(run(() -> Runnable)).withTimeout(1)
     *     /// This results in an InstantCommand ending instantly.
     *     .andThen(() -> Runnable);
     * </code></pre>
     *
     * @return a runnable that runs the state.
     */
    protected abstract Command collectState();
    // TODO update state command commenting
    /**
     * The score state to run.
     *
     * <p>
     *
     * <h1>Commands should be structured like below to prevent InstantCommands, causing the opponent to cycle states
     * without the command finishing.</h1>
     *
     * <p>
     *
     * <p>
     *
     * <h1>Good </h1>
     *
     * <pre><code>
     *     pathfind(getRandomFromMap(config.getPoseMap)).withTimeout(7) // Go to element
     *     .andThen(manipulatorSim.intake("Intake").withTimeout(0.5)) // Run intake
     *     .andThen(manipulatorSim.intake("Intake") // Run intake
     *              .withDeadline(Commands.waitSeconds(1)) // Add deadline timer to only intake()
     *     .andThen(runOnce(() -> Runnable)) // Stop something
     *     .andThen(Commands.waitSeconds(0.5)) // Wait before continuing
     *     .finallyDo(() -> setState("Score")); // Now cycle to next state.
     * </code></pre>
     *
     * <p>
     *
     * <p>
     *
     * <h1>Bad </h1>
     *
     * <pre><code>
     *     pathfind
     *     /// This adds the command then adds the timeout to the entire command sequence.
     *     .andThen(run(() -> Runnable)).withTimeout(1)
     *     /// This results in an InstantCommand ending instantly.
     *     .andThen(() -> Runnable);
     * </code></pre>
     *
     * @return a runnable that runs the state.
     */
    protected abstract Command scoreState();

    @Override
    public void simulationPeriodic() {
        boolean commandInProgress =
                getCurrentCommand() != null && !getCurrentCommand().isFinished();
        if (!commandInProgress && !Objects.equals("Standby", config.desiredState)) {
            runState(config.desiredState, false).schedule();
        }
        statePublisher.set(config.currentState);
        posePublisher.set(drivetrainSim.getActualPoseInSimulationWorld());
    }

    /**
     * Sets the current state of the robot. This waits its turn patiently for the command to finish.
     *
     * @param state The state to set.
     * @return this, for chaining.
     */
    protected SmartOpponent setState(String state) {
        config.desiredState = state;
        return this;
    }

    /**
     * Runs a state as a command.
     *
     * @param state The state to run.
     * @param forceState Whether to force the state to run even if it is already running.
     * @return The command to run the state.
     */
    protected Command runState(String state, boolean forceState) {
        // If forceState, cancel any commands.
        if (forceState) {
            Command currentCommand = getCurrentCommand();
            if (currentCommand != null) {
                currentCommand.cancel();
            }
            return config.getStates().get(state).get();
        }
        // Don't force the state. If there's a command running or already in state, wait.
        if (config.currentState.equals(state)
                || getCurrentCommand() != null
                || (getCurrentCommand() != null && !getCurrentCommand().isFinished())
                        && (!RobotModeTriggers.disabled().getAsBoolean() && config.isAutoEnable)) {
            setState(state); // Make state wait for command to finish.
            return Commands.none();
        }
        // Nothing in the way, get our state.
        return config.getStates().get(state).get();
    }

    /**
     * Gets the current actual opponent pose.
     *
     * @return the opponent {@link Pose2d}.
     */
    public Pose2d getOpponentPose() {
        return drivetrainSim.getActualPoseInSimulationWorld();
    }

    /**
     * Gets the opponent's current active target pose.
     *
     * @return either the opponent target pose or null if there is no active target.
     */
    public Pair<String, Pose2d> getTarget() {
        return target;
    }

    /**
     * Pathfinds to a target pose.
     *
     * @param target The target.
     * @return A command to pathfind to the target pose.
     */
    protected Command pathfind(Pair<String, Pose2d> target, Time timeout) {
        /// Determine pose data
        // Store targetPose as a global var
        this.target = Pair.of(target.getFirst(), ifShouldFlip(target.getSecond()));
        // Add offset after setting flipped generic target.
        final Pose2d finalPose = this.target.getSecond().plus(config.pathfindOffset);
        // Initialize our desiredState once.
        final PathPlannerTrajectoryState desiredState = new PathPlannerTrajectoryState();
        /// Set up the pathfinder
        mapleADStar.setStartPosition(
                drivetrainSim.getActualPoseInSimulationWorld().getTranslation());
        mapleADStar.setGoalPosition(finalPose.getTranslation());
        mapleADStar.runThread();
        /// Defer the command to runtime, this updates our values when the command is called upon.
        /// This makes sure all of our data is fresh as can be.
        return Commands.defer(
                () -> {
                    // Store initial waypoint count
                    final int[] initialWaypointCount = {mapleADStar.currentWaypoints.size()};
                    return Commands.run(() -> {
                                /// If we are close to another opponent registered by our manager, update obstacles and
                                // recalculate our path.
                                if (manager != null) {
                                    ifNearAddObstacles(mapleADStar, this, Meters.of(1.5));
                                }
                                final Pose2d currentPose = drivetrainSim.getActualPoseInSimulationWorld();
                                final List<Waypoint> waypoints = mapleADStar.currentWaypoints;
                                Pose2d currentTarget;
                                /// If waypoints exist, make the next one our target.
                                if (!waypoints.isEmpty()) {
                                    currentTarget = new Pose2d(
                                            waypoints.get(0).anchor(),
                                            // Interpolate our rotation goal for smooth rotation. Rotation is 0 on these
                                            // obtained goals.
                                            currentPose
                                                    .getRotation()
                                                    .interpolate(
                                                            finalPose.getRotation(),
                                                            (double) (initialWaypointCount[0] - waypoints.size())
                                                                    / initialWaypointCount[0]));
                                    /// If we are close enough, remove it and move to the next waypoint.
                                    if (nearPose(
                                            currentTarget,
                                            Meters.of(1), // How close we should be to the next anchor before moving to
                                            // the next anchor
                                            Degrees.of(
                                                    180))) // How correct our angle should be before moving to the next
                                    // anchor
                                    // We don't need to force any angle here for swerve, it l
                                    {
                                        waypoints.remove(0);
                                    }
                                } /// Else, we set our target to our desired final pose.
                                else {
                                    currentTarget = finalPose;
                                }
                                /// Create a new desired state
                                desiredState.pose = currentTarget;
                                desiredState.heading = currentTarget.getRotation();
                                /// Calculate our chassis speeds.
                                driveController.reset(
                                        drivetrainSim.getActualPoseInSimulationWorld(),
                                        drivetrainSim.getActualSpeedsFieldRelative());
                                ChassisSpeeds speeds =
                                        driveController.calculateRobotRelativeSpeeds(currentPose, desiredState);
                                drivetrainSim.runChassisSpeeds(speeds, new Translation2d(), false, false);
                            }) /// Once we have no more targets, finish the command.
                            .until(() -> {
                                List<Waypoint> waypoints = mapleADStar.currentWaypoints;
                                return waypoints.isEmpty()
                                        && nearPose(
                                                finalPose,
                                                config.chassis.driveToPoseTolerance,
                                                config.chassis.driveToPoseAngleTolerance);
                            }) /// Finally, stop the chassis and clear our target.
                            .finallyDo(() -> {
                                drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false);
                                this.target = Pair.of("", Pose2d.kZero);
                            }) /// Set a timeout, usually 7 sec is good.,
                            .withTimeout(timeout);
                },
                /// A set of this subsystem for the deferred command requirements.
                Set.of(this));
    }

    /**
     * A {@link Command} to drive the {@link SmartOpponent}. Meant to be used for joystick drives.
     *
     * @param chassisSpeeds speeds supplier to run the robot.
     * @return {@link Command} to drive the {@link SmartOpponent}.
     */
    protected Command drive(Supplier<ChassisSpeeds> chassisSpeeds, boolean fieldCentric) {
        if (fieldCentric) {
            return run(() -> {
                // Blue alliance faces 180°, Red faces 0°
                Rotation2d driverFacing =
                        config.alliance.equals(DriverStation.Alliance.Blue) ? Rotation2d.k180deg : Rotation2d.kZero;

                // Convert to field-centric and run
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds.get(), driverFacing);
                drivetrainSim.runChassisSpeeds(fieldSpeeds, new Translation2d(), true, false);
            });
        }

        // Robot-relative
        return run(() -> drivetrainSim.runChassisSpeeds(chassisSpeeds.get(), new Translation2d(), false, false));
    }

    /**
     * Makes a list of bounding boxes for obstacle avoidance.
     *
     * @return a list of obstacles usable by {@link MapleADStar}.
     */
    public List<Pair<Translation2d, Translation2d>> getObstacles() {
        return getObstaclesDynamicPoll(Seconds.of(0));
    }

    /**
     * Saves a list globally in {@link OpponentManager} and returns that list only updating it if the last update
     * exceeds the given time.
     *
     * @param updateTime how long to wait before updating.
     * @return a list of obstacles usable by {@link MapleADStar}.
     */
    public List<Pair<Translation2d, Translation2d>> getObstaclesDynamicPoll(Time updateTime) {
        if (System.currentTimeMillis() - lastObstaclePoll > updateTime.in(Millisecond)) {
            lastObstaclePoll = System.currentTimeMillis();
            latestObstacles.clear();
            manager.getOpponents().forEach(opponent -> {
                Translation2d pose = opponent.getOpponentPose().getTranslation();
                latestObstacles.add(
                        Pair.of(pose.plus(manager.boundingBoxTranslation), pose.minus(manager.boundingBoxTranslation)));
            });
        }
        return latestObstacles;
    }

    /**
     * Adds obstacles to the given pathfinder if the given opponent is near another opponent. Used in the
     * {@link SmartOpponent} pathfinding command.
     *
     * @param pathfinder the pathfinder to update.
     * @param currentOpponent the opponent to check.
     * @param tolerance how far from other opponents should we be.
     * @return whether the thread was run.
     */
    public boolean ifNearAddObstacles(MapleADStar pathfinder, SmartOpponent currentOpponent, Distance tolerance) {
        final int[] runThread = {0};
        final Pose2d currentPose = currentOpponent.getOpponentPose();
        final Translation2d currentTranslation = currentPose.getTranslation();
        // Check all opponents that aren't the current opponent
        manager.getOpponentPoses().stream()
                .filter(opponentPose ->
                        opponentPose.getTranslation().getDistance(currentTranslation) < tolerance.in(Meters))
                .filter(opponentPose -> opponentPose.getTranslation().getDistance(currentTranslation) > 0.2)
                .forEach(opponent -> {
                    pathfinder.setDynamicObstacles(
                            getObstaclesDynamicPoll(Seconds.of(0.2)),
                            currentOpponent.getOpponentPose().getTranslation());
                    runThread[0] = 1;
                });
        // If we should run the thread, run it.
        if (runThread[0] == 1) {
            pathfinder.setStartPosition(currentTranslation);
            pathfinder.runThread();
        }
        // Return if we ran the thread.
        return runThread[0] == 1;
    }

    /**
     * Gets a random pose from a map. If the map is empty reports an error and returns null.
     *
     * @param poseMap The map to get a pose from.
     * @return A random pose from the map.
     */
    protected Pair<String, Pose2d> getRandomFromMap(Map<String, Pose2d> poseMap) {
        Map.Entry<String, Pose2d> randomEntry = poseMap.entrySet().stream()
                .skip(new Random().nextInt(poseMap.size()))
                .findFirst()
                .orElse(null);

        return randomEntry != null ? Pair.of(randomEntry.getKey(), randomEntry.getValue()) : null;
    }

    /**
     * Gets a random pose from a map.TODO If the map is empty reports an error and returns null.
     *
     * @param poseMap The map to get a pose from.
     * @return A random pose from the map.
     */
    protected Pair<String, Pose2d> getTargetFromMap(Map<String, Pose2d> poseMap) {
        final Map<String, Pose2d> newMap = new HashMap<>(poseMap);
        newMap.values().removeAll(manager.getOpponentTargets());
        return newMap.isEmpty() ? null : getRandomFromMap(newMap);
    }

    /**
     * TODO
     *
     * @return
     */
    protected Pair<String, Pose2d> getScoringTarget() {
        // If no entries exist, fail.
        if (config.cachedScoringEntries.isEmpty()) {
            return null;
        }

        // Cache opponent targets and weight map to avoid repeated calls.
        final var opponentTargets = manager.getOpponentTargets();
        final var scoringWeights = config.getScoringWeights();
        double random = Math.random();
        double cumulativeWeight = 0;
        double totalWeight = 0;

        // First pass: calculate total weight of available entries.
        for (var entry : config.cachedScoringEntries) {
            if (!opponentTargets.contains(entry.getValue())) {
                totalWeight += scoringWeights.getOrDefault(entry.getKey(), 1.0);
            }
        }

        // If no available targets, fail.
        if (totalWeight == 0) {
            return null;
        }

        // Scale random value by total weight for weighted selection.
        random *= totalWeight;

        // Second pass: find weighted target using cumulative weights.
        for (var entry : config.cachedScoringEntries) {
            if (!opponentTargets.contains(entry.getValue())) {
                cumulativeWeight += scoringWeights.getOrDefault(entry.getKey(), 1.0);
                if (random <= cumulativeWeight) {
                    return Pair.of(entry.getKey(), entry.getValue());
                }
            }
        }

        // No target found (shouldn't happen if totalWeight > 0).
        return null;
    }

    /**
     * TODO
     * @return
     */
    protected Pair<String, Pose2d> getCollectTarget() {
        // If no entries exist, fail.
        if (config.cachedCollectingEntries.isEmpty()) {
            return null;
        }

        // Cache opponent targets and weight map to avoid repeated calls.
        final var opponentTargets = manager.getOpponentTargets();
        final var collectWeights = config.getCollectWeights();
        double random = Math.random();
        double cumulativeWeight = 0;
        double totalWeight = 0;

        // First pass: calculate total weight of available entries.
        for (var entry : config.cachedCollectingEntries) {
            if (!opponentTargets.contains(entry.getValue())) {
                totalWeight += collectWeights.getOrDefault(entry.getKey(), 1.0);
            }
        }

        // If no available targets, fail.
        if (totalWeight == 0) {
            return null;
        }

        // Scale random value by total weight for weighted selection.
        random *= totalWeight;

        // Second pass: find weighted target using cumulative weights.
        for (var entry : config.cachedCollectingEntries) {
            if (!opponentTargets.contains(entry.getValue())) {
                cumulativeWeight += collectWeights.getOrDefault(entry.getKey(), 1.0);
                if (random <= cumulativeWeight) {
                    return Pair.of(entry.getKey(), entry.getValue());
                }
            }
        }

        // No target found (shouldn't happen if totalWeight > 0).
        return null;
    }

    /**
     * Checks if the {@link SmartOpponent} is near the given pose within a given tolerance.
     *
     * @param pose the pose to check against.
     * @param tolerance the translation tolerance in {@link Distance}.
     * @param angleTolerance the rotation tolerance in {@link Angle}.
     * @return
     */
    public boolean nearPose(Pose2d pose, Distance tolerance, Angle angleTolerance) {
        Pose2d robotPose = drivetrainSim.getActualPoseInSimulationWorld();
        return robotPose.getTranslation().getDistance(pose.getTranslation()) < tolerance.in(Meters)
                && Math.abs(robotPose.getRotation().minus(pose.getRotation()).getDegrees())
                        < angleTolerance.in(Degrees);
    }

    /**
     * Checks if the {@link SmartOpponent} is moving within the given tolerance. This checks only the linear velocity.
     *
     * @param tolerance the movement speed tolerance in {@link LinearVelocity}.
     * @return true if the robot is moving faster then the given tolerance.
     */
    public boolean isMoving(LinearVelocity tolerance) {
        return drivetrainSim.getActualSpeedsRobotRelative().vxMetersPerSecond > tolerance.in(MetersPerSecond)
                || drivetrainSim.getActualSpeedsRobotRelative().vyMetersPerSecond > tolerance.in(MetersPerSecond);
    }

    /**
     * If robot is red, returns pose as is. If the robot is blue, it returns the flipped pose.
     *
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    protected Pose2d ifShouldFlip(Pose2d pose) {
        if (config.alliance == DriverStation.Alliance.Blue) {
            return pose;
        } else {
            return new Pose2d(
                    FieldMirroringUtils.flip(pose.getTranslation()), FieldMirroringUtils.flip(pose.getRotation()));
        }
    }
}
