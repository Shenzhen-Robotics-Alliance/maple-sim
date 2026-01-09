package org.ironmaple.simulation.opponentsim;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.*;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * The config is required to make a {@link SmartOpponent}.
 * It should throw if configured wrong.
 */
public class SmartOpponentConfig {
    /// Basic Required Options
    public String name;
    public DriverStation.Alliance alliance;
    public Pose2d queeningPose;
    public Pose2d initialPose;
    /// Basic Optional Options
    // Telemetry Options
    public String telemetryPath;
    public String smartDashboardPath;
    protected final SendableChooser<Command> behaviorChooser;
    // Joystick Options
    protected Optional<Object> joystick = Optional.empty();
    public double joystickDeadband = 0.15;
    // Pathfind pose offset translation to compensate for different chassis'
    public Transform2d pathfindOffset;
    // Auto Enable and Disable Opponent Support
    public boolean isAutoEnable;

    /// Opponent Chassis Options
    public ChassisConfig chassis;

    /// Maps using a (key, value) pair. Used to
    // Map of the robot states.
    private final Map<String, Supplier<Command>> states = new HashMap<>();
    // Map of behaviour options
    private final Map<String, Pair<Command, Boolean>> behavior = new HashMap<>();
    // Map of possible scoring poses and types. For example, Map<"Hoops", Map<"CourtLeft", Pose2d>>
    private final Map<String, Map<String, Pose2d>> scoringMap = new HashMap<>();
    // Map of possible collecting poses and types. For example, Map<"CollectStation", Map<"StationCenter", Pose2d>>
    private final Map<String, Map<String, Pose2d>> collectingMap = new HashMap<>();
    // Map of scoring pose weights, if a weight is not defined, it defaults to 1.0.
    private final Map<String, Double> scoringWeightMap = new HashMap<>();
    // Map of collect pose weights, if a weight is not defined, it defaults to 1.0.
    private final Map<String, Double> collectWeightMap = new HashMap<>();
    /// Cached weighted maps for faster targeting.
    // Cached entries for each pose.
    public final List<Map.Entry<String, Pose2d>> cachedScoringEntries = new ArrayList<>();
    // Cached cumulative weights for a binary search.
    public final List<Double> cachedScoringWeights = new ArrayList<>();
    // Cached entries for each pose.
    public final List<Map.Entry<String, Pose2d>> cachedCollectingEntries = new ArrayList<>();
    // Cached cumulative weights for a binary search.
    public final List<Double> cachedCollectingWeights = new ArrayList<>();

    /// Opponent Management
    protected OpponentManager manager;
    protected Time pollRate;
    public boolean commandInProgress = false;
    // Opponents current and last states.
    public String currentState = "";
    public String desiredState;

    /// Sets of required basic options
    private final Set<BasicOptions> requiredBasicOptions;

    /** Default Constructor for SmartOpponentConfig. */
    public SmartOpponentConfig() {
        /// Sets the required basic options.
        this.requiredBasicOptions = EnumSet.allOf(BasicOptions.class);
        withPathfindOffset(Transform2d.kZero); // Sets a zero offset for default use.
        withPollRate(SimulatedArena.getSimulationDt());

        /// Prepares empty config.
        this.chassis = new ChassisConfig();

        /// Prepares Telemetry
        withTelemetryPath("MapleSim/");
        this.behaviorChooser = new SendableChooser<>();
    }

    public void loadWeightedPoses() {
        // Reset and load scoring poses
        cachedScoringEntries.clear();
        cachedScoringEntries.addAll(getScoringMap().entrySet());
        double totalScoringWeight = 0.0;

        /// Cache our targets with a weight
        for (Map.Entry<String, Double> entry : scoringWeightMap.entrySet()) {
            // If no weight is set, default to 1.0
            double weight = scoringWeightMap.getOrDefault(entry.getKey(), 1.0);
            totalScoringWeight += weight;
            // Save our cumulative weight for a binary search
            cachedScoringWeights.add(totalScoringWeight);
        }

        // Reset and load collect poses
        cachedCollectingEntries.clear();
        cachedCollectingEntries.addAll(getCollectingMap().entrySet());
        double totalCollectWeight = 0.0;
        /// Cache our targets with a weight
        for (Map.Entry<String, Double> entry : collectWeightMap.entrySet()) {
            // If no weight is set, default to 1.0
            double weight = collectWeightMap.getOrDefault(entry.getKey(), 1.0);
            totalCollectWeight += weight;
            // Save our cumulative weight for a binary search
            cachedCollectingWeights.add(totalCollectWeight);
        }
    }

    /**
     * Constructor for SmartOpponentConfig with all required options set.
     *
     * @param name The robot's name.
     * @param alliance The robot's alliance.
     * @param initialPose The robot's initial pose.
     * @param queeningPose The robot's queening pose.
     * @param chassisConfig The robot's chassis config.
     */
    public SmartOpponentConfig(
            String name,
            DriverStation.Alliance alliance,
            Pose2d initialPose,
            Pose2d queeningPose,
            ChassisConfig chassisConfig) {
        /// Creates a new config using the empty base.
        this();
        /// Add required options to the checklist.
        this.withName(name)
                .withAlliance(alliance)
                .withQueeningPose(queeningPose)
                .withStartingPose(initialPose)
                .withChassisConfig(chassisConfig);
    }

    /**
     * Sets the robot name.
     *
     * @param name The robot name.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withName(String name) {
        this.name = name;
        this.requiredBasicOptions.remove(BasicOptions.Name);
        return this;
    }

    /**
     * Sets the robot alliance.
     *
     * @param alliance The robot alliance.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
        this.requiredBasicOptions.remove(BasicOptions.Alliance);
        return this;
    }

    /**
     * Adds a {@link OpponentManager} to register with. Set alliance and name beforehand. Sets these values as well
     * withStartingPose() withQueeningPose() withCollectingMap() withScoringMap()
     *
     * @param manager the {@link OpponentManager} to use.
     * @return this, for chaining.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withManager(OpponentManager manager) {
        if (alliance == null) {
            DriverStation.reportError("Alliance is null", true);
        }
        this.manager = manager;
        withStartingPose(this.manager.getInitialPose(alliance))
                .withQueeningPose(this.manager.getQueeningPose())
                .withCollectingMap(this.manager.getRawCollectingMap())
                .withScoringMap(this.manager.getRawScoringMap());
        return this;
    }

    /**
     * Gets this opponent's manager if registered.
     *
     * @return the {@link OpponentManager} being used.
     */
    public OpponentManager getManager() {
        if (manager == null) {
            DriverStation.reportWarning("MapleSim getManager() returned null, no manager was found.", false);
            return null;
        }
        return manager;
    }

    /**
     * Sets the queening pose.
     *
     * @param pose The pose to set the robot to.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withQueeningPose(Pose2d pose) {
        this.queeningPose = pose;
        this.requiredBasicOptions.remove(BasicOptions.QueeningPose);
        return this;
    }

    /**
     * Sets the initial pose.
     *
     * @param initialPose The initial pose.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withStartingPose(Pose2d initialPose) {
        this.initialPose = initialPose;
        this.requiredBasicOptions.remove(BasicOptions.InitialPose);
        return this;
    }

    /**
     * Sets the telemetry path.
     *
     * @param telemetryPath The telemetry path.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withTelemetryPath(String telemetryPath) {
        // Force a smartdashboard path since it's needed for the SendableChooser.
        this.telemetryPath = "SmartDashboard/" + telemetryPath;
        this.smartDashboardPath = telemetryPath;
        return this;
    }

    /**
     * Sets the joystick to use. May or may not be used depending on the opponent's setup.
     *
     * @param joystick The joystick to use.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withJoystick(Object joystick) {
        this.joystick = Optional.of(joystick);
        return this;
    }

    /**
     * Gets the optional joystick {@link Object}. Needs cast properly for use.
     *
     * @return the optional joystick {@link Object}.
     */
    public Object getJoystick() {
        if (joystick.isPresent()) {
            return joystick.get();
        }
        DriverStation.reportError("Joystick not found, returning null. Joystick is: " + joystick, true);
        return null;
    }

    /**
     * Sets the joystick deadband.
     *
     * @param deadband The joystick deadband.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withJoystickDeadband(double deadband) {
        this.joystickDeadband = deadband;
        return this;
    }

    /**
     * Sets the Pathfinding offset.
     *
     * @param pathfindOffset value to transform targets by.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withPathfindOffset(Transform2d pathfindOffset) {
        this.pathfindOffset = pathfindOffset;
        return this;
    }

    /**
     * This sets a dynamic poll rate for various obstacle and pose getters.
     *
     * @param pollRate how long to wait before refreshing our getters.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withPollRate(Time pollRate) {
        this.pollRate = pollRate;
        return this;
    }

    /**
     * Adds a scoring weight to a target pose.
     * The weight is 1 by default, raise it to go there more often.
     *
     * @param weights the map of weights to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withScoringPoseWeights(Map<String, Double> weights) {
        this.scoringWeightMap.putAll(weights);
        return this;
    }

    /**
     * Adds a scoring weight to a target pose.
     * The weight is 1 by default, raise it to go there more often.
     *
     * @param name the pose to add it to.
     * @param weight the weight to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withScoringPoseWeights(String name, double weight) {
        this.scoringWeightMap.put(name, weight);
        return this;
    }

    /**
     * Gets the map of scoring weights.
     *
     * @return the map of scoring weights.
     */
    public Map<String, Double> getScoringWeights() {
        return scoringWeightMap;
    }

    /**
     * Adds a scoring weight to a target pose.
     * The weight is 1 by default, raise it to go there more often.
     *
     * @param weights the map of weights to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withCollectPoseWeights(Map<String, Double> weights) {
        this.collectWeightMap.putAll(weights);
        return this;
    }

    /**
     * Adds a scoring weight to a target pose.
     * The weight is 1 by default, raise it to go there more often.
     *
     * @param name the pose to add it to.
     * @param weight the weight to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withCollectPoseWeights(String name, double weight) {
        this.collectWeightMap.put(name, weight);
        return this;
    }

    /**
     * Gets the map of collect weights.
     *
     * @return the map of collect weights.
     */
    public Map<String, Double> getCollectWeights() {
        return collectWeightMap;
    }

    /**
     * Sets the chassis config. This is the only input to encourage full configuration of the robot.
     *
     * @param chassisConfig The chassis config to set.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withChassisConfig(ChassisConfig chassisConfig) {
        this.chassis = chassisConfig;
        return this;
    }

    /**
     * Enables automatically updating behavior with the RobotBase
     *
     * @return this, for chaining.
     */
    public SmartOpponentConfig withAutoEnable() {
        this.isAutoEnable = true;
        return this;
    }

    /**
     * Setting for automatically updating behavior with the RobotBase
     *
     * @param autoEnable whether to enable.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withAutoEnable(boolean autoEnable) {
        this.isAutoEnable = autoEnable;
        return this;
    }

    /**
     * Sets the current state of the robot. This waits it's turn patiently for the command to finish.
     *
     * @param state The state to set.
     * @return this, for chaining.
     */
    public SmartOpponentConfig setState(String state) {
        desiredState = state;
        return this;
    }

    /**
     * Adds a state to the config.
     *
     * @param stateName The name of the state.
     * @param stateCommand The command to run when the state is entered.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withState(String stateName, Supplier<Command> stateCommand) {
        states.put(stateName, () -> stateCommand
                .get()
                .beforeStarting(() -> {
                    setState(stateName);
                    currentState = stateName;
                    commandInProgress = true;
                })
                .finallyDo(() -> {
                    commandInProgress = false;
                }));
        return this;
    }

    /**
     * Removes a state from the config.
     *
     * @param stateName The name of the state to remove.
     * @return this, for chaining.
     */
    public SmartOpponentConfig removeState(String stateName) {
        states.remove(stateName);
        return this;
    }

    /**
     * Replaces a state's command.
     *
     * @param stateName The name of the state to replace.
     * @param newStateCommand The new command to run when the state is entered.
     * @return this, for chaining.
     */
    public SmartOpponentConfig replaceStateCommand(String stateName, Supplier<Command> newStateCommand) {
        states.replace(stateName, () -> newStateCommand
                .get()
                .beforeStarting(() -> {
                    setState(stateName);
                    currentState = stateName;
                    commandInProgress = true;
                })
                .finallyDo(() -> {
                    commandInProgress = false;
                }));
        return this;
    }

    /**
     * Returns the state map.
     *
     * @return the state map.
     */
    public Map<String, Supplier<Command>> getStates() {
        return states;
    }

    /**
     * Checks if the current state is equal to the given state.
     *
     * @param state the state to check.
     * @return return true if states are the same.
     */
    public boolean isState(String state) {
        return currentState.equals(state);
    }

    /**
     * Checks if the current state is equal to the given state.
     *
     * @param state the state to check.
     * @return return true if states are the same.
     */
    public Trigger isStateTrigger(String state) {
        return new Trigger(() -> currentState.equals(state));
    }

    /**
     * Adds a behavior to the config.
     *
     * @param behaviorName The name of the behavior.
     * @param behaviorCommand The command to run when the behavior is entered.
     * @param isDefault whether this option should be the default. There can only be one.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withBehavior(String behaviorName, Command behaviorCommand, boolean isDefault) {
        behavior.put(behaviorName, Pair.of(behaviorCommand, isDefault));
        return this;
    }

    /**
     * Adds a non-default behavior to the config.
     *
     * @param behaviorName The name of the behavior.
     * @param behaviorCommand The command to run when the behavior is entered.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withBehavior(String behaviorName, Command behaviorCommand) {
        return withBehavior(behaviorName, behaviorCommand, false);
    }

    /**
     * Removes a behavior from the config.
     *
     * @param behaviorName The name of the behavior to remove.
     * @return this, for chaining.
     */
    public SmartOpponentConfig removeBehavior(String behaviorName) {
        behavior.remove(name);
        return this;
    }

    /**
     * Replaces a behavior's command.
     *
     * @param behaviorName The name of the behavior to replace.
     * @param newBehaviorCommand The new command to run when the behavior is entered.
     * @param isDefault whether this option should be the default. There can only be one.
     * @return this, for chaining.
     */
    public SmartOpponentConfig replaceBehaviourCommand(
            String behaviorName, Command newBehaviorCommand, boolean isDefault) {
        behavior.put(name, Pair.of(newBehaviorCommand, isDefault));
        return this;
    }

    /**
     * Returns the behavior map.
     *
     * @return the behavior map.
     */
    public Map<String, Pair<Command, Boolean>> getBehavior() {
        return behavior;
    }

    /**
     * Updates and gets the refreshed behavior {@link SendableChooser<Command>}.
     *
     * @return the refreshed behavior {@link SendableChooser<Command>}.
     */
    public SendableChooser<Command> updateBehaviorChooser() {
        boolean hasDefault = false;
        /// Load all behaviours
        for (Map.Entry<String, Pair<Command, Boolean>> entry : behavior.entrySet()) {
            // if behavior wants to be default.
            if (entry.getValue().getSecond()) {
                // Check if there is already a default.
                if (hasDefault) {
                    // If there's multiple defaults report it.
                    DriverStation.reportError(
                            "Behaviour Chooser has multiple defaults, ignoring future defaults.", false);
                    // Then add it as normal
                    behaviorChooser.addOption(entry.getKey(), entry.getValue().getFirst());
                } else {
                    // If there are no issues, add the default.
                    behaviorChooser.setDefaultOption(
                            entry.getKey(), entry.getValue().getFirst());
                    hasDefault = true;
                }
            } else {
                // Behavior isn't default, add it like normal.
                behaviorChooser.addOption(entry.getKey(), entry.getValue().getFirst());
            }
        }
        return behaviorChooser;
    }

    /**
     * Gets the behavior {@link SendableChooser<Command>}.
     *
     * @return the behavior {@link SendableChooser<Command>}.
     */
    public SendableChooser<Command> getBehaviorChooser() {
        return this.behaviorChooser;
    }

    /**
     * Adds all poses from an existing pose map.
     *
     * @param scoringMap the map to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withScoringMap(Map<String, Map<String, Pose2d>> scoringMap) {
        this.scoringMap.putAll(scoringMap);
        return this;
    }

    /**
     * Adds a scoring pose to the config.
     *
     * @param poseType The type of pose to add. For example, "Hoops".
     * @param poseName The name of the pose to add. For example, "CourtLeft".
     * @param pose The pose to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withScoringPose(String poseType, String poseName, Pose2d pose) {
        scoringMap.putIfAbsent(poseType, new HashMap<>());
        scoringMap.get(poseType).putIfAbsent(poseName, pose);
        if (scoringMap.get(poseType).get(poseName) != pose) {
            throw new IllegalArgumentException("Failed to add score pose: " + poseName + "/n");
        }
        return this;
    }

    /**
     * Removes a scoring pose from the config.
     *
     * @param poseType The type of pose to remove.
     * @param poseName The name of the pose to remove.
     * @return this, for chaining.
     */
    public SmartOpponentConfig removeScoringPose(String poseType, String poseName) {
        scoringMap.get(poseType).remove(poseName);
        return this;
    }

    /**
     * Removes all scoring poses of a certain type from the config.
     *
     * @param poseType The type of poses to remove.
     * @return this, for chaining.
     */
    public SmartOpponentConfig removeScoringPoseType(String poseType) {
        scoringMap.remove(poseType);
        return this;
    }

    /**
     * Replaces a scoring pose.
     *
     * @param poseType The type of pose to replace.
     * @param poseName The name of the pose to replace.
     * @param pose The pose to replace with.
     * @return this, for chaining.
     */
    public SmartOpponentConfig replaceScoringPose(String poseType, String poseName, Pose2d pose) {
        scoringMap.get(poseType).replace(poseName, pose);
        return this;
    }

    /**
     * Gets the raw scoring map. Setup as Map<poseType, Map<poseName, Pose2d>>
     *
     * @return the raw scoring map.
     */
    public Map<String, Map<String, Pose2d>> getRawScoringMap() {
        return scoringMap;
    }

    /**
     * Compiles a compacted scoring map. Setup as Map<poseName, Pose2d>
     *
     * @return a compacted scoring map.
     */
    public Map<String, Pose2d> getScoringMap() {
        Map<String, Pose2d> scoringPoses = new HashMap<>();
        scoringMap.forEach((poseType, poseMap) -> poseMap.forEach((name, pose) -> {
            scoringPoses.put(poseType + " " + name, pose);}));
        return scoringPoses;
    }

    /**
     * Compiles a list of all scoring poses.
     *
     * @return a list of all scoring poses.
     */
    public List<Pose2d> getScoringPoses() {
        List<Pose2d> scoringPoses = new ArrayList<>();
        scoringMap.forEach((poseType, poseMap) -> poseMap.forEach((poseName, pose) -> scoringPoses.add(pose)));
        return scoringPoses;
    }

    /**
     * Adds all poses from an existing pose map.
     *
     * @param collectingMap the map to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withCollectingMap(Map<String, Map<String, Pose2d>> collectingMap) {
        this.collectingMap.putAll(collectingMap);
        return this;
    }

    /**
     * Adds a collecting pose to the config.
     *
     * @param poseType The type of pose to add. For example, "CollectStation".
     * @param poseName The name of the pose to add. For example, "StationCenter".
     * @param pose The pose to add.
     * @return this, for chaining.
     */
    public SmartOpponentConfig withCollectingPose(String poseType, String poseName, Pose2d pose) {
        collectingMap.putIfAbsent(poseType, new HashMap<>());
        collectingMap.get(poseType).putIfAbsent(poseName, pose);
        if (collectingMap.get(poseType).get(poseName) != pose) {
            throw new IllegalArgumentException("Failed to add collect pose: " + poseName + "/n");
        }
        return this;
    }

    /**
     * Removes a collecting pose from the config.
     *
     * @param poseType The type of pose to remove.
     * @param poseName The name of the pose to remove.
     * @return this, for chaining.
     */
    public SmartOpponentConfig removeCollectingPose(String poseType, String poseName) {
        collectingMap.get(poseType).remove(poseName);
        return this;
    }

    /**
     * Removes all collecting poses of a certain type from the config.
     *
     * @param poseType The type of poses to remove.
     * @return this, for chaining.
     */
    public SmartOpponentConfig removeCollectingPoseType(String poseType) {
        collectingMap.remove(poseType);
        return this;
    }

    /**
     * Replaces a collecting pose.
     *
     * @param poseType The type of pose to replace.
     * @param poseName The name of the pose to replace.
     * @param pose The pose to replace with.
     * @return this, for chaining.
     */
    public SmartOpponentConfig replaceCollectingPose(String poseType, String poseName, Pose2d pose) {
        collectingMap.get(poseType).replace(poseName, pose);
        return this;
    }

    /**
     * Gets the raw collecting map. Setup as Map<poseType, Map<poseName, Pose2d>>
     *
     * @return the raw collecting map.
     */
    public Map<String, Map<String, Pose2d>> getRawCollectingMap() {
        return collectingMap;
    }

    /**
     * Compiles a compacted collecting map. Setup as Map<poseName, Pose2d>
     *
     * @return a compacted collecting map.
     */
    public Map<String, Pose2d> getCollectingMap() {
        Map<String, Pose2d> collectingPoses = new HashMap<>();
        collectingMap.forEach((poseType, poseMap) -> poseMap.forEach((name, pose) ->
                collectingPoses.put(poseType + " " + name, pose)));
        return collectingPoses;
    }

    /**
     * Compiles a list of all collecting poses.
     *
     * @return a list of all collecting poses.
     */
    public List<Pose2d> getCollectingPoses() {
        List<Pose2d> collectingPoses = new ArrayList<>();
        collectingMap.forEach((poseType, poseMap) -> poseMap.forEach((poseName, pose) -> collectingPoses.add(pose)));
        return collectingPoses;
    }

    /**
     * Checks if the config is valid.
     *
     * @return true if the config is valid, false otherwise.
     * @throws IllegalArgumentException if any config is incorrect.
     */
    public boolean validateConfigs() {
        boolean validConfig = chassis.validConfig() && requiredBasicOptions.isEmpty();
        // If not set correct, report settings to update. Then throw an error.
        if (!validConfig) {
            for (BasicOptions option : requiredBasicOptions) {
                DriverStation.reportWarning(
                        "Required Option: " + option.toString() + " not set in SmartOpponentConfig", true);
            }
            throw new IllegalArgumentException(
                    "Invalid SmartOpponentConfig, check DriverStation Output for more info.");
        }
        // If any required options aren't removed, return false.
        return validConfig;
    }

    /** Basic Options that MUST be set for the {@link SmartOpponentConfig} to be valid. */
    private enum BasicOptions {
        /// Required Basic Options
        /// Robot Name
        Name,
        /// Robot Alliance
        Alliance,
        /// Queening Pose
        QueeningPose,
        /// Starting Pose
        InitialPose
    }

    /**
     * The SmartOpponent's Chassis Config, this is an inner config that is only chassis configuration.
     */
    public static class ChassisConfig {
        /// Required chassis options to be set later.
        public Distance trackWidth;
        public Distance trackLength;
        public Distance bumperWidth;
        public Distance bumperLength;
        public Mass mass;
        public MomentOfInertia moi;
        public LinearVelocity maxLinearVelocity;
        public AngularVelocity maxAngularVelocity;
        public ModuleConfig module;
        /// Optional chassis options
        public Distance driveToPoseTolerance;
        public Angle driveToPoseAngleTolerance;
        public Supplier<GyroSimulation> gyroSimulation;
        /// Sets of required options set here.
        public Set<ChassisOptions> requiredChassisOptions;
        public LinearAcceleration maxLinearAcceleration;
        public AngularAcceleration maxAngularAcceleration;
        // Saved simulation for later use
        private SwerveModuleSimulationConfig moduleSimConfig;
        public RobotConfig pathplannerConfig;

        /** Default constructor for ChassisConfig. Each Option must be set individually. */
        ChassisConfig() {
            /// Adds the required options to the checklist.
            this.requiredChassisOptions = EnumSet.allOf(ChassisOptions.class);
            /// Optional chassis options
            this.driveToPoseTolerance = Inches.of(2);
            this.driveToPoseAngleTolerance = Degrees.of(6);
            this.gyroSimulation = COTS.ofGenericGyro();
        }

        /**
         * Constructor for ChassisConfig.
         *
         * @param trackWidth the distance from one center of a rear wheel to the other.
         * @param trackHeight the distance from one center of a left or right wheel to the other.
         * @param bumperWidth the distance from the left of the robot to the right with bumpers.
         * @param bumperHeight the distance from the front of the robot to the rear with the bumpers.
         * @param mass the mass of the robot.
         * @param moi the moment of inertia of the robot.
         * @param maxLinearVelocity the max linear velocity of the robot.
         * @param maxLinearAcceleration the max linear acceleration of the robot.
         * @param maxAngularVelocity the max angular velocity of the robot.
         * @param maxAngularAcceleration the max angular acceleration of the robot.
         * @param module the {@link ModuleConfig} for the robot's swerve modules.
         */
        ChassisConfig(
                Distance trackWidth,
                Distance trackHeight,
                Distance bumperWidth,
                Distance bumperHeight,
                Mass mass,
                MomentOfInertia moi,
                LinearVelocity maxLinearVelocity,
                LinearAcceleration maxLinearAcceleration,
                AngularVelocity maxAngularVelocity,
                AngularAcceleration maxAngularAcceleration,
                ModuleConfig module) {
            /// Creates a new config using the empty base.
            this();
            /// Add required options to the checklist.
            this.withTrackWidth(trackWidth)
                    .withTrackLength(trackHeight)
                    .withBumperWidth(bumperWidth)
                    .withBumperHeight(bumperHeight)
                    .withMass(mass)
                    .withMOI(moi)
                    .withMaxLinearVelocity(maxLinearVelocity)
                    .withMaxLinearAcceleration(maxLinearAcceleration)
                    .withMaxAngularVelocity(maxAngularVelocity)
                    .withMaxAngularAcceleration(maxAngularAcceleration)
                    .withModule(module);
        }

        /**
         * Constructor for ChassisConfig.
         *
         * @param mass The mass of the robot.
         * @param moi The moment of inertia of the robot.
         * @param maxLinearVelocity The max linear velocity of the robot.
         * @param maxAngularVelocity The max angular velocity of the robot.
         * @param wheelRadius the module's wheel radius.
         * @param wheelCOF the module's drive Coefficient of Friction.
         * @param driveMotor the module's drive motor.
         * @param driveCurrentLimit the module's drive current limit.
         */
        ChassisConfig(
                Distance trackWidth,
                Distance trackHeight,
                Distance bumperWidth,
                Distance bumperHeight,
                Mass mass,
                MomentOfInertia moi,
                LinearVelocity maxLinearVelocity,
                LinearAcceleration maxLinearAcceleration,
                AngularVelocity maxAngularVelocity,
                AngularAcceleration maxAngularAcceleration,
                Distance wheelRadius,
                double wheelCOF,
                DCMotor driveMotor,
                DCMotor steerMotor,
                double driveGearRatio,
                double steerGearRatio,
                Voltage driveFrictionVoltage,
                Voltage steerFrictionVoltage,
                MomentOfInertia steerAngularInertia,
                LinearVelocity maxDriveVelocity,
                Current driveCurrentLimit) {
            /// Creates a new config from the previous constructor with a manually set ModuleConfig
            this(
                    trackWidth,
                    trackHeight,
                    bumperWidth,
                    bumperHeight,
                    mass,
                    moi,
                    maxLinearVelocity,
                    maxLinearAcceleration,
                    maxAngularVelocity,
                    maxAngularAcceleration,
                    new ModuleConfig(
                            wheelRadius,
                            wheelCOF,
                            driveMotor,
                            steerMotor,
                            driveGearRatio,
                            steerGearRatio,
                            driveFrictionVoltage,
                            steerFrictionVoltage,
                            steerAngularInertia,
                            maxDriveVelocity,
                            driveCurrentLimit));
        }

        /**
         * Creates a new SwerveDriveSimulation from the current ChassisConfig.
         *
         * @param initialPose the initial pose of the robot.
         * @return a new SwerveDriveSimulation from the current ChassisConfig.
         */
        public SelfControlledSwerveDriveSimulation createDriveTrainSim(Pose2d initialPose) {
            if (!validConfig()) {
                throw new IllegalStateException("Config is not valid! Cannot create the driveTrain simulation!");
            }
            if (module.driveMotor == null || module.steerMotor == null) {
                throw new IllegalStateException(
                        "ModuleConfig has null motors! Drive: " + module.driveMotor + ", Steer: " + module.steerMotor);
            }
            return new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                    new DriveTrainSimulationConfig(
                            mass,
                            bumperLength,
                            bumperWidth,
                            trackLength,
                            trackWidth,
                            gyroSimulation,
                            () -> new SwerveModuleSimulation(moduleSimConfig) // Force new sims from the config.
                            // Disable global battery draw quick fix.
                            ),
                    initialPose));
        }

        /**
         * Creates a new RobotConfig from the current ChassisConfig.
         *
         * @return the new RobotConfig.
         */
        public RobotConfig updatePathplannerConfig() {
            this.pathplannerConfig = new RobotConfig(mass, moi, module.updatePathplannerModuleConfig(), trackWidth);
            return this.pathplannerConfig;
        }

        /**
         * Checks if the config is valid.
         *
         * @return true if the config is valid, false otherwise.
         */
        public boolean validConfig() {
            boolean validConfig = module.validConfig() && requiredChassisOptions.isEmpty();
            if (!validConfig) {
                for (ChassisOptions option : requiredChassisOptions) {
                    DriverStation.reportWarning(
                            "Required Option: " + option.toString() + " not set in ChassisConfig", false);
                }
            }
            // If any required options aren't removed, return false.
            return validConfig;
        }

        /**
         * Sets the track width of the robot. This is from one center of the wheel to the other.
         *
         * @param trackWidth The track width of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withTrackWidth(Distance trackWidth) {
            this.trackWidth = trackWidth;
            this.requiredChassisOptions.remove(ChassisOptions.TrackWidth);
            return this;
        }

        /**
         * Sets the track height of the robot.
         *
         * @param trackHeight The track height of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withTrackLength(Distance trackHeight) {
            this.trackLength = trackHeight;
            this.requiredChassisOptions.remove(ChassisOptions.TrackLength);
            return this;
        }

        /**
         * Sets the bumper width of the robot.
         *
         * @param bumperWidth The bumper width of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withBumperWidth(Distance bumperWidth) {
            this.bumperWidth = bumperWidth;
            this.requiredChassisOptions.remove(ChassisOptions.BumperWidth);
            return this;
        }

        /**
         * Sets the bumper height of the robot.
         *
         * @param bumperHeight The bumper height of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withBumperHeight(Distance bumperHeight) {
            this.bumperLength = bumperHeight;
            this.requiredChassisOptions.remove(ChassisOptions.BumperLength);
            return this;
        }

        /**
         * Simplified Chassis Configuration for a square robot. Sets the bumper width and height to the same value.
         *
         * @param bumperLength the length of the bumpers.
         * @param trackLength the track width of the robot. This is from one center of the wheel to the other.
         * @return this, for chaining.
         */
        public ChassisConfig withSquareChassis(Distance bumperLength, Distance trackLength) {
            return this.withBumperWidth(bumperLength)
                    .withBumperHeight(bumperLength)
                    .withTrackWidth(trackLength)
                    .withTrackLength(trackLength);
        }

        /**
         * Sets the mass of the robot.
         *
         * @param mass The mass of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withMass(Mass mass) {
            this.mass = mass;
            this.requiredChassisOptions.remove(ChassisOptions.Mass);
            return this;
        }

        /**
         * Sets the moment of inertia of the robot.
         *
         * @param moi The moment of inertia of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withMOI(MomentOfInertia moi) {
            this.moi = moi;
            this.requiredChassisOptions.remove(ChassisOptions.MOI);
            return this;
        }

        /**
         * Sets the max linear velocity of the robot.
         *
         * @param maxLinearVelocity The max linear velocity of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withMaxLinearVelocity(LinearVelocity maxLinearVelocity) {
            this.maxLinearVelocity = maxLinearVelocity;
            this.requiredChassisOptions.remove(ChassisOptions.MaxLinearVelocity);
            return this;
        }

        /**
         * Sets the max linear acceleration of the robot.
         *
         * @param maxLinearAcceleration The max linear acceleration of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withMaxLinearAcceleration(LinearAcceleration maxLinearAcceleration) {
            this.maxLinearAcceleration = maxLinearAcceleration;
            this.requiredChassisOptions.remove(ChassisOptions.MaxLinearAcceleration);
            return this;
        }

        /**
         * Sets the max angular velocity of the robot.
         *
         * @param maxAngularVelocity The max angular velocity of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withMaxAngularVelocity(AngularVelocity maxAngularVelocity) {
            this.maxAngularVelocity = maxAngularVelocity;
            this.requiredChassisOptions.remove(ChassisOptions.MaxAngularVelocity);
            return this;
        }

        /**
         * Sets the max angular acceleration of the robot.
         *
         * @param maxAngularAcceleration The max angular acceleration of the robot.
         * @return this, for chaining.
         */
        public ChassisConfig withMaxAngularAcceleration(AngularAcceleration maxAngularAcceleration) {
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.requiredChassisOptions.remove(ChassisOptions.MaxAngularAcceleration);
            return this;
        }

        /**
         * Sets the module config.
         *
         * @param module The module config.
         * @return this, for chaining.
         */
        public ChassisConfig withModule(ModuleConfig module) {
            this.module = module;
            this.moduleSimConfig = module.createModuleSimConfig();
            this.requiredChassisOptions.remove(ChassisOptions.ModuleConfig);
            return this;
        }

        /**
         * Sets the tolerance for the drive to pose command.
         *
         * @param tolerance the tolerance for the drive to pose command.
         * @return this, for chaining.
         */
        public ChassisConfig withDriveToPoseTolerance(Distance tolerance) {
            this.driveToPoseTolerance = tolerance;
            return this;
        }

        public ChassisConfig withDriveToPoseAngleTolerance(Angle tolerance) {
            this.driveToPoseAngleTolerance = tolerance;
            return this;
        }

        /**
         * Sets the gyro simulation for the robot.
         *
         * @param gyroSimulation
         * @return
         */
        public ChassisConfig withGyroSimulation(Supplier<GyroSimulation> gyroSimulation) {
            this.gyroSimulation = gyroSimulation;
            return this;
        }

        /** Chassis Options that MUST be set for the {@link SmartOpponentConfig} to be valid. */
        public enum ChassisOptions {
            /// Required Opponent Chassis Options
            /// Opponent Robot Track Width
            TrackWidth,
            /// Opponent Robot Track Height
            TrackLength,
            /// Opponent Robot Bumper Width
            BumperWidth,
            /// Opponent Robot Bumper Height
            BumperLength,
            /// Opponent Robot Mass
            Mass,
            /// Opponent Robot Moment of Inertia
            MOI,
            /// Opponent Robot Max Linear Velocity
            MaxLinearVelocity,
            /// Opponent Robot Max Linear Acceleration
            MaxLinearAcceleration,
            /// Opponent Robot Max Angular Velocity
            MaxAngularVelocity,
            /// Opponent Robot Max Angular Acceleration
            MaxAngularAcceleration,
            /// Opponent Module Config
            ModuleConfig
        }

        /**
         * A few chassis presets to simplify configs that don't need to be specific.
         */
        public enum Presets {
            /// <p> Opponent Swerve Chassis config presets to ease creation. </p>
            /// A simple config that defaults harder values to viable values.
            SimpleSetup,
            /// A basic square robot. Made to match the AdvantageScope basic assets.
            SimpleSquareChassis;

            /**
             * Returns a new ChassisConfig for the preset. Each call returns a fresh, independent instance.
             *
             * @return a new ChassisConfig for the preset.
             */
            public ChassisConfig getConfig() {
                return switch (this) {
                    case SimpleSetup -> new ChassisConfig()
                            .withMOI(KilogramSquareMeters.of(8))
                            .withModule(ModuleConfig.Presets.SimpleSetup.getConfig());
                    case SimpleSquareChassis -> new ChassisConfig()
                            .withSquareChassis(Inches.of(32), Inches.of(23)) // 32x32 bumper, 24 track width
                            .withMass(Kilograms.of(55))
                            .withMOI(KilogramSquareMeters.of(8))
                            .withMaxLinearVelocity(MetersPerSecond.of(8.5))
                            .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(12))
                            .withMaxAngularVelocity(DegreesPerSecond.of(360))
                            .withMaxAngularAcceleration(DegreesPerSecondPerSecond.of(480))
                            .withModule(ModuleConfig.Presets.MK4i.getConfig());
                };
            }
        }

        /**
         * Loads another config into this config.
         *
         * @param other the config to copy.
         * @return return the updated config.
         */
        public ChassisConfig copy(ChassisConfig other) {
            this.withTrackWidth(other.trackWidth)
                    .withTrackLength(other.trackLength)
                    .withBumperWidth(other.bumperWidth)
                    .withBumperHeight(other.bumperLength)
                    .withMass(other.mass)
                    .withMOI(other.moi)
                    .withMaxLinearVelocity(other.maxLinearVelocity)
                    .withMaxAngularVelocity(other.maxAngularVelocity)
                    .withModule(ModuleConfig.clone(other.module));
            return this;
        }

        /**
         * Creates a new clone of a config.
         *
         * @param other the config to clone.
         * @return the newly cloned config.
         */
        public static ChassisConfig clone(ChassisConfig other) {
            return new ChassisConfig().copy(other);
        }
    }

    /**
     * The SmartOpponent's Module Config, this is an inner config that is only the module configuration.
     */
    public static class ModuleConfig {
        /// Constants to be initialized in the constructor.
        public Distance wheelRadius;
        public double wheelCOF;
        public DCMotor driveMotor;
        public DCMotor steerMotor;
        public double driveGearRatio;
        public double steerGearRatio;
        public Voltage driveFrictionVoltage;
        public Voltage steerFrictionVoltage;
        public MomentOfInertia steerAngularInertia;
        public LinearVelocity maxDriveVelocity;
        public Current driveCurrentLimit;
        /// Set of required options for the ModuleConfig
        public Set<ModuleOptions> requiredModuleOptions;
        // Saved objects for later use.
        public SwerveModuleSimulation moduleSim;
        public com.pathplanner.lib.config.ModuleConfig pathplannerModuleConfig;

        /** Empty constructor for ModuleConfig. Each Option must be set individually. */
        ModuleConfig() {
            /// Add required options to the checklist.
            this.requiredModuleOptions = EnumSet.allOf(ModuleOptions.class);
        }

        /**
         * Constructor for ModuleConfig.
         *
         * @param wheelRadius the wheel radius.
         * @param wheelCOF the wheel Coefficient of Friction.
         * @param driveMotor the drive {@link DCMotor}.
         * @param driveCurrentLimit the drive {@link Current} limit.
         */
        ModuleConfig(
                Distance wheelRadius,
                double wheelCOF,
                DCMotor driveMotor,
                DCMotor steerMotor,
                double driveGearRatio,
                double steerGearRatio,
                Voltage driveFrictionVoltage,
                Voltage steerFrictionVoltage,
                MomentOfInertia steerAngularInertia,
                LinearVelocity maxDriveVelocity,
                Current driveCurrentLimit) {
            /// Creates a new config using the empty base.
            this();
            /// Add required options to the checklist.
            this.withWheelRadius(wheelRadius)
                    .withWheelCOF(wheelCOF)
                    .withDriveMotor(driveMotor)
                    .withSteerMotor(steerMotor)
                    .withDriveGearRatio(driveGearRatio)
                    .withSteerGearRatio(steerGearRatio)
                    .withDriveFrictionVoltage(driveFrictionVoltage)
                    .withSteerFrictionVoltage(steerFrictionVoltage)
                    .withSteerAngularInertia(steerAngularInertia)
                    .withMaxDriveVelocity(maxDriveVelocity)
                    .withDriveCurrentLimit(driveCurrentLimit);
        }

        /**
         * Creates a new {@link SwerveModuleSimulation} from a {@link com.pathplanner.lib.config.ModuleConfig}. Gets
         * saved for later use.
         *
         * @return the new {@link SwerveModuleSimulation}.
         */
        public SwerveModuleSimulationConfig createModuleSimConfig() {
            return new SwerveModuleSimulationConfig(
                    driveMotor,
                    steerMotor,
                    driveGearRatio,
                    steerGearRatio,
                    driveFrictionVoltage,
                    steerFrictionVoltage,
                    wheelRadius,
                    steerAngularInertia,
                    wheelCOF);
        }

        /**
         * Creates a new {@link com.pathplanner.lib.config.ModuleConfig} from a
         * {@link com.pathplanner.lib.config.ModuleConfig}.
         *
         * @return the new {@link com.pathplanner.lib.config.ModuleConfig}.
         */
        public com.pathplanner.lib.config.ModuleConfig updatePathplannerModuleConfig() {
            this.pathplannerModuleConfig = new com.pathplanner.lib.config.ModuleConfig(
                    wheelRadius,
                    maxDriveVelocity,
                    wheelCOF,
                    driveMotor,
                    driveCurrentLimit,
                    1); // Hard set until someone complains
            return this.pathplannerModuleConfig;
        }

        /**
         * Checks if the config is valid.
         *
         * @return true if the config is valid, false otherwise.
         */
        public boolean validConfig() {
            boolean validConfig = requiredModuleOptions.isEmpty();
            if (!validConfig) {
                for (ModuleOptions option : requiredModuleOptions) {
                    DriverStation.reportError(
                            "Required Option: " + option.toString() + " not set in ModuleConfig", false);
                }
            }
            return validConfig;
        }

        /**
         * Sets the wheel radius.
         *
         * @param wheelRadius The wheel radius.
         * @return this, for chaining.
         */
        public ModuleConfig withWheelRadius(Distance wheelRadius) {
            this.wheelRadius = wheelRadius;
            this.requiredModuleOptions.remove(ModuleOptions.WheelRadius);
            return this;
        }

        /**
         * Sets the wheel Coefficient of Friction.
         *
         * @param wheelCOF The wheel Coefficient of Friction.
         * @return this, for chaining.
         */
        public ModuleConfig withWheelCOF(double wheelCOF) {
            this.wheelCOF = wheelCOF;
            this.requiredModuleOptions.remove(ModuleOptions.WheelCOF);
            return this;
        }

        /**
         * Sets the drive {@link DCMotor}.
         *
         * @param driveMotor The drive {@link DCMotor}.
         * @return this, for chaining.
         */
        public ModuleConfig withDriveMotor(DCMotor driveMotor) {
            this.driveMotor = driveMotor;
            this.requiredModuleOptions.remove(ModuleOptions.DriveMotor);
            return this;
        }

        /**
         * Sets the steer {@link DCMotor}.
         *
         * @param dcMotor The steer {@link DCMotor}.
         * @return this, for chaining.
         */
        public ModuleConfig withSteerMotor(DCMotor dcMotor) {
            this.steerMotor = dcMotor;
            this.requiredModuleOptions.remove(ModuleOptions.SteerMotor);
            return this;
        }

        /**
         * Sets the gear ratios for the drive motor.
         *
         * @param driveGearRatio The gear ratio for the drive motor.
         * @return this, for chaining.
         */
        public ModuleConfig withDriveGearRatio(double driveGearRatio) {
            this.driveGearRatio = driveGearRatio;
            //this.driveMotor = driveMotor.withReduction(driveGearRatio);
            this.requiredModuleOptions.remove(ModuleOptions.DriveRatio);
            return this;
        }

        /**
         * Sets the gear ratio for the steer motor.
         *
         * @param steerGearRatio The gear ratio for the steer motor.
         * @return this, for chaining.
         */
        public ModuleConfig withSteerGearRatio(double steerGearRatio) {
            this.steerGearRatio = steerGearRatio;
            //this.steerMotor = steerMotor.withReduction(steerGearRatio);
            this.requiredModuleOptions.remove(ModuleOptions.SteerRatio);
            return this;
        }

        /**
         * Sets the friction voltage for the drive motor.
         *
         * @param driveFrictionVoltage The friction voltage for the drive motor.
         * @return this, for chaining.
         */
        public ModuleConfig withDriveFrictionVoltage(Voltage driveFrictionVoltage) {
            this.driveFrictionVoltage = driveFrictionVoltage;
            this.requiredModuleOptions.remove(ModuleOptions.DriveFrictionVoltage);
            return this;
        }

        /**
         * Sets the friction voltage for the steer motor.
         *
         * @param steerFrictionVoltage The friction voltage for the steer motor.
         * @return this, for chaining.
         */
        public ModuleConfig withSteerFrictionVoltage(Voltage steerFrictionVoltage) {
            this.steerFrictionVoltage = steerFrictionVoltage;
            this.requiredModuleOptions.remove(ModuleOptions.SteerFrictionVoltage);
            return this;
        }

        /**
         * Sets the steer angular inertia.
         *
         * @param steerAngularInertia The steer angular inertia.
         * @return this, for chaining.
         */
        public ModuleConfig withSteerAngularInertia(MomentOfInertia steerAngularInertia) {
            this.steerAngularInertia = steerAngularInertia;
            this.requiredModuleOptions.remove(ModuleOptions.SteerAngularInertia);
            return this;
        }

        /**
         * Sets the max drive velocity.
         *
         * @param maxDriveVelocity The max drive velocity.
         * @return this, for chaining.
         */
        public ModuleConfig withMaxDriveVelocity(LinearVelocity maxDriveVelocity) {
            this.maxDriveVelocity = maxDriveVelocity;
            this.requiredModuleOptions.remove(ModuleOptions.MaxDriveVelocity);
            return this;
        }

        /**
         * Sets the drive {@link Current} limit.
         *
         * @param driveCurrentLimit The drive {@link Current} limit.
         * @return this, for chaining.
         */
        public ModuleConfig withDriveCurrentLimit(Current driveCurrentLimit) {
            this.driveCurrentLimit = driveCurrentLimit;
            this.requiredModuleOptions.remove(ModuleOptions.DriveCurrentLimit);
            return this;
        }

        /** Module Options that MUST be set for the {@link SmartOpponentConfig} to be valid. */
        public enum ModuleOptions {
            /// Required Opponent Drive Module Options
            /// Opponent Drive Module Wheel Radius
            WheelRadius,
            /// Opponent Drive Wheel Coefficient of Friction
            WheelCOF,
            /// Opponent Drive DCMotor
            DriveMotor,
            /// Opponent Steer DCMotor
            SteerMotor,
            /// Opponent Drive Ratio
            DriveRatio,
            /// Opponent Steer Ratio
            SteerRatio,
            /// Opponent Drive Friction Voltage
            DriveFrictionVoltage,
            /// Opponent Steer Friction Voltage
            SteerFrictionVoltage,
            /// Opponent Steer Angular Inertia
            SteerAngularInertia,
            /// Opponent Drive Motor Max Velocity
            MaxDriveVelocity,
            /// Opponent Drive Module Current Limit
            DriveCurrentLimit
        }

        /**
         * A few module presets to simplify configs that don't need to be specific.
         */
        public enum Presets {
            /// <p> Opponent Swerve module config presets to ease creation. </p>
            /// A simple config that defaults harder values to viable values.
            SimpleSetup,
            /// WCP MK4i Swerve Module with Neo Drive motor.
            MK4i;

            /**
             * Returns a new ModuleConfig for the preset. Each call returns a fresh, independent instance.
             *
             * @return a new ModuleConfig for the preset.
             */
            public ModuleConfig getConfig() {
                return switch (this) {
                    case SimpleSetup -> new ModuleConfig()
                            .withWheelRadius(Inches.of(2))
                            .withWheelCOF(1.19)
                            .withDriveFrictionVoltage(Volts.of(0.3))
                            .withSteerFrictionVoltage(Volts.of(0.2))
                            .withDriveCurrentLimit(Amps.of(40));
                    case MK4i -> new ModuleConfig()
                            .withWheelRadius(Inches.of(2))
                            .withWheelCOF(1.19)
                            .withDriveMotor(DCMotor.getNEO(1))
                            .withSteerMotor(DCMotor.getNEO(1))
                            .withDriveGearRatio(6.75) // L2 Ratio
                            .withSteerGearRatio(150 / 7.0) // MK4i Steer Ratio Calculation
                            .withDriveFrictionVoltage(Volts.of(0.3))
                            .withSteerFrictionVoltage(Volts.of(0.2))
                            .withSteerAngularInertia(KilogramSquareMeters.of(0.03))
                            .withMaxDriveVelocity(MetersPerSecond.of(10))
                            .withDriveCurrentLimit(Amps.of(40));
                };
            }
        }

        /**
         * Copies the ModuleConfig from another ModuleConfig.
         *
         * @param other the other ModuleConfig to copy from.
         * @return this, for chaining.
         */
        public ModuleConfig copy(ModuleConfig other) {
            withWheelRadius(other.wheelRadius);
            withWheelCOF(other.wheelCOF);
            withDriveMotor(other.driveMotor);
            withSteerMotor(other.steerMotor);
            withDriveGearRatio(other.driveGearRatio);
            withSteerGearRatio(other.steerGearRatio);
            withDriveFrictionVoltage(other.driveFrictionVoltage);
            withSteerFrictionVoltage(other.steerFrictionVoltage);
            withSteerAngularInertia(other.steerAngularInertia);
            withMaxDriveVelocity(other.maxDriveVelocity);
            withDriveCurrentLimit(other.driveCurrentLimit);
            return this;
        }

        /**
         * Constructs a new {@link ModuleConfig} from a {@link ModuleConfig}.
         *
         * @param other the other ModuleConfig to copy from.
         * @return the new ModuleConfig.
         */
        public static ModuleConfig clone(ModuleConfig other) {
            return new ModuleConfig().copy(other);
        }
    }
}
