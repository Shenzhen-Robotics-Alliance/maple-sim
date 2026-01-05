package org.ironmaple.simulation.opponentsim;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    protected ManipulatorSim manipulatorSim;
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
    // Whether the state was interrupted and should restart.
    protected boolean restartInterrupt;
    /// Collision Detection
    protected final Debouncer collisionDebouncer;
    /// Not Moving Detection
    protected final Timer notMovingTimer;
    protected final LinearVelocity notMovingThreshold;

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
        this.driveController = new PPHolonomicDriveController(
                new PIDConstants(5),
                new PIDConstants(5));
        // Cloned Pathfinder for use here.
        this.mapleADStar = new MapleADStar();
        // Preset an empty manipulator
        this.manipulatorSim = new ManipulatorSim();
        /// Initialize simulations
        this.drivetrainSim = config.chassis.createDriveTrainSim(config.queeningPose);
        this.pathplannerConfig = config.chassis.updatePathplannerConfig();
        // Alliance string for telemetry.
        this.allianceString = DriverStation.Alliance.Blue.equals(config.alliance) ? "Blue Alliance/" : "Red Alliance/";
        // NetworkTable setup.
        this.statePublisher = NetworkTableInstance.getDefault()
                .getStringTopic(config.telemetryPath + "SimulatedOpponents/States/"
                        + allianceString + config.name + "'s Current State").publish();
        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(config.telemetryPath + "SimulatedOpponents/Poses/"
                        + allianceString + config.name + "'s Pose2d", Pose2d.struct).publish();
        this.selectedBehaviorPublisher = NetworkTableInstance.getDefault()
                .getTable(config.telemetryPath + "SimulatedOpponents/Behaviors/"
                        + allianceString + config.name + "'s Behaviors")
                .getStringTopic("selected").publish();
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
                config.smartDashboardPath
                        + "SimulatedOpponents/Behaviors/"
                        + allianceString + config.name
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
        // Initialize an empty target.
        this.target = Pair.of("None", Pose2d.kZero);
        /// Collision Detection
        this.collisionDebouncer = new Debouncer(1, Debouncer.DebounceType.kRising);
        /// Not Moving Detection
        this.notMovingThreshold = FeetPerSecond.of(1);
        this.notMovingTimer = new Timer();
        // Caches weighted poses for a faster search later.
        config.loadWeightedPoses();
    }

    /**
     * Returns whether the opponent is on the given alliance.
     *
     * @param alliance the alliance to check.
     * @return whether the opponent is on the given alliance.
     */
    public boolean isAlliance(DriverStation.Alliance alliance) {
        return this.config.alliance == alliance;
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
        return run(() -> {
            drivetrainSim.setSimulationWorldPose(config.queeningPose);
            drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false);
            restartInterrupt = false;
        }).ignoringDisable(true);
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
        // If command not in progress and standby isn't the desired state. Or if a restartInterrupt occurred, restarting the current state.
        if (!config.commandInProgress && !Objects.equals("Standby", config.desiredState)) {
            runState(config.desiredState, false).schedule();
        }
        // If restart requested, do that now.
        if (restartInterrupt) {
            runState(config.desiredState, true).schedule();
            notMovingTimer.restart();
            restartInterrupt = false;
        }
        // If we are very stuck reload from start.
        if (notMovingTimer.hasElapsed(5)) {
            drivetrainSim.setSimulationWorldPose(config.initialPose);
            runState(config.desiredState, true).schedule();
        }
        // If the timer is not running and a command is running and the opponent is not moving, start the timer.
        if (!notMovingTimer.isRunning() && config.commandInProgress && !isMoving(notMovingThreshold)) {
            notMovingTimer.restart();
        } else {
            notMovingTimer.stop();
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

    protected boolean isColliding() {
        final var collisionFactor =
                LinearFilter.movingAverage(50).calculate(Math.abs(
                        (Arrays.stream(drivetrainSim.getDriveTrainSimulation().getModules()).findFirst().get().getDriveMotorSupplyCurrent()).in(Amps)
                                - Arrays.stream(drivetrainSim.getDriveTrainSimulation().getModules()).findAny().get().getDriveMotorAppliedVoltage().in(Volts)));
        return collisionDebouncer.calculate(MathUtil.isNear(0.6, collisionFactor, 0.3));
    }

    /**
     * Pathfinds to a target pose.
     *
     * @param target The target.
     * @return A command to pathfind to the target pose.
     */
    public Command pathfind(Pair<String, Pose2d> target, LinearVelocity desiredVelocity) {
        if (Objects.isNull(target)) {
            System.out.println(config.name + " Opponent's target is null, skipping this cycle.");
            return Commands.none();
        }
        this.target = Pair.of(target.getFirst(), ifShouldFlip(target.getSecond()));
        // Add offset after setting flipped generic target.
        final Pose2d finalPose = this.target.getSecond().plus(config.pathfindOffset);
        /// Set up the pathfinder
        mapleADStar.setStartPosition(drivetrainSim.getActualPoseInSimulationWorld().getTranslation());
        mapleADStar.setGoalPosition(finalPose.getTranslation());
        mapleADStar.runThread();
        return Commands.run(() -> {
                    // Initialize our poses and targets
                    final var currentPose = drivetrainSim.getActualPoseInSimulationWorld();
                    final var waypoints = mapleADStar.currentWaypoints;
                    final Translation2d targetTranslation;
                    final Rotation2d targetRotation;
                    // If waypoints exist, load our next target.
                    if (!waypoints.isEmpty()) {
                        // Set anchor as a target.
                        targetTranslation = waypoints.get(0).anchor();
                        // Incorrectly interpolate rotation goal.
                        targetRotation = currentPose.getRotation().interpolate(finalPose.getRotation(), waypoints.size());
                        // If close to current target transfer to the next.
                        if (currentPose.getTranslation().getDistance(targetTranslation) < config.chassis.driveToPoseTolerance.times(3).in(Meters)) {
                            waypoints.remove(0);
                        }
                        // If no waypoints, set the final pose as the target.
                    } else {
                        targetTranslation = finalPose.getTranslation();
                        targetRotation = finalPose.getRotation();
                    }
                    final var targetPose = new Pose2d(targetTranslation, targetRotation);
                    final PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
                    state.pose = targetPose;
                    // Run out calculated speeds.
                    drivetrainSim.runChassisSpeeds(
                            driveController.calculateRobotRelativeSpeeds(
                                    currentPose,
                                    state), new Translation2d(), false, false);
                }, this)
                .until(() -> { // Run until no waypoints and within tolerances.
                    List<Waypoint> waypoints = mapleADStar.currentWaypoints;
                    return waypoints.isEmpty() && nearPose(finalPose, config.chassis.driveToPoseTolerance, config.chassis.driveToPoseAngleTolerance);
                }) // If an opponent seems stuck, restart.
                .until(() -> {
                    final boolean bool = notMovingFor(Seconds.of(0.5)) || isColliding();
                    if (bool) {
                        notMovingTimer.restart();
                        restartInterrupt = true;
                    }
                    return bool;
                })
                // When finished, stop moving.
                .finallyDo(() -> drivetrainSim.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false));
    }

    /**
     * TODO
     * @param target
     * @return
     */
    protected Command followPath(Pair<String, PathPlannerPath> target) {
        return new FollowPathCommand(
                target.getSecond(),
                () -> drivetrainSim.getActualPoseInSimulationWorld(),
                () -> drivetrainSim.getActualSpeedsRobotRelative(),
                (speeds, ffNotUsed) -> drivetrainSim.runChassisSpeeds(speeds, new Translation2d(), false, false),
                driveController,
                pathplannerConfig,
                () -> config.alliance == DriverStation.Alliance.Blue,
                this);
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
     * Gets a random pose from a map.
     * If the map is empty returns null.
     *
     * @param poseMap The map to get a pose from.
     * @return A random pose from the map, unless empty.
     */
    protected Pair<String, Pose2d> getTargetFromMap(Map<String, Pose2d> poseMap) {
        final Map<String, Pose2d> newMap = new HashMap<>(poseMap);
        // Remove all already targeted poses.
        newMap.values().removeAll(manager.getOpponentTargetsDynamic(config.pollRate)); // ignore warning, if it doesn't exist or match, nothing will be removed.
        // Return null if empty, otherwise continue.
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
        final var opponentTargets = manager.getOpponentTargetsDynamic(config.alliance, config.pollRate);
        final var scoringWeights = config.getScoringWeights();
        double random = Math.random();
        double cumulativeWeight = 0;
        double totalWeight = 0;

        // First pass: calculate total weight of available entries.
        for (var entry : config.cachedScoringEntries) {
            // Check both that the target is not already taken and not near any existing targets
            if (!opponentTargets.contains(entry.getValue()) &&
                    !manager.isNearTarget(entry.getValue(), config.pollRate, Meters.of(1.0))) {
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

        final var opponentTargets = manager.getOpponentTargetsDynamic(config.alliance, config.pollRate);
        final var collectWeights = config.getCollectWeights();
        double random = Math.random();
        double cumulativeWeight = 0;
        double totalWeight = 0;

        // First pass: calculate total weight of available entries.
        for (var entry : config.cachedCollectingEntries) {
            // Check both that the target is not already taken and not near any existing targets
            if (!opponentTargets.contains(entry.getValue()) &&
                    !manager.isNearTarget(entry.getValue(), config.pollRate, Meters.of(1.0))) {
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
     * @return true if the robot is moving faster than the given tolerance.
     */
    public boolean isMoving(LinearVelocity tolerance) {
        final var speeds = drivetrainSim.getActualSpeedsRobotRelative();
        return speeds.vxMetersPerSecond > tolerance.in(MetersPerSecond)
                || speeds.vyMetersPerSecond > tolerance.in(MetersPerSecond);
    }

    /**
     * Checks if the {@link SmartOpponent} has not been moving for given time.
     *
     * @param elapsedTime how long the robot should be stuck for.
     * @return true if the robot has been stopped for the elaspedTime.
     */
    public boolean notMovingFor(Time elapsedTime) {
        return notMovingTimer.hasElapsed(elapsedTime.in(Seconds)) && !isMoving(notMovingThreshold);
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
