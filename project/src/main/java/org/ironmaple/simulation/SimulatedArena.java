package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h1>Abstract Simulation World</h1>
 *
 * <p>Check <a href='https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/'>Online
 * Documentation</a>
 *
 * <h2>The heart of the simulator.</h2>
 *
 * <p>This class cannot be instantiated directly; it must be created as a specific season's arena.
 *
 * <p>The default instance can be obtained using the {@link #getInstance()} method.
 *
 * <p>Simulates all interactions within the arena field.
 *
 * <h2>The following objects can be added to the simulation world and will interact with each other: </h2>
 *
 * <ul>
 *   <li>{@link AbstractDriveTrainSimulation}: Represents abstract drivetrain simulations with collision detection.
 *   <li>{@link GamePieceOnFieldSimulation}: Represents abstract game pieces with collision detection.
 *   <li>{@link IntakeSimulation}: Represents an intake simulation that responds to contact with
 *       {@link GamePieceOnFieldSimulation}.
 * </ul>
 */
public abstract class SimulatedArena {
    /** Whether to allow the simulation to run a real robot This feature is HIGHLY RECOMMENDED to be turned OFF */
    public static boolean ALLOW_CREATION_ON_REAL_ROBOT = false;

    protected int redScore = 0;
    protected int blueScore = 0;
    protected double matchClock = 0;

    public Map<String, Double> redScoringBreakdown = new Hashtable<String, Double>();
    public Map<String, Double> blueScoringBreakdown = new Hashtable<String, Double>();
    protected Map<String, DoublePublisher> redPublishers = new Hashtable<String, DoublePublisher>();
    protected Map<String, DoublePublisher> bluePublishers = new Hashtable<String, DoublePublisher>();

    public NetworkTable redTable =
            NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance");
    public NetworkTable blueTable =
            NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim/MatchData/Breakdown/blue Alliance");
    public NetworkTable genericInfoTable =
            NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim/MatchData/Breakdown");

    public DoublePublisher matchClockPublisher =
            genericInfoTable.getDoubleTopic("Match Clock").publish();

    public static BooleanPublisher resetFieldPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData")
            .getBooleanTopic("Reset Field")
            .publish();

    public static BooleanSubscriber resetFieldSubscriber =
            resetFieldPublisher.getTopic().subscribe(false);

    Boolean shouldPublishMatchBreakdown = true;

    private static SimulatedArena instance = null;

    /**
     *
     *
     * <h2>Gets/Creates the Default Simulation World</h2>
     *
     * <p>Multiple instances of {@link SimulatedArena} can exist elsewhere.
     *
     * @return the main simulation arena instance
     * @throws IllegalStateException if the method is call when running on a real robot
     */
    public static SimulatedArena getInstance() {
        if (RobotBase.isReal() && (!ALLOW_CREATION_ON_REAL_ROBOT))
            throw new IllegalStateException(
                    "MapleSim is running on a real robot! (If you would actually want that, set SimulatedArena.ALLOW_CREATION_ON_REAL_ROBOT to true).");

        if (instance == null) instance = new org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt();

        return instance;
    }

    /**
     *
     *
     * <h2>Overrides the Default Simulation World</h2>
     *
     * <p>Overrides the return value of {@link #getInstance()}
     *
     * <p>This method allows simulating an arena from a different year or a custom field.
     *
     * <p>Currently, only the 2024 arena is supported, so avoid calling this method for now.
     *
     * @param newInstance the new simulation arena instance to override the current one
     */
    public static void overrideInstance(SimulatedArena newInstance) {
        if (instance != null) instance = new Arena2026Rebuilt();
        instance = newInstance;
    }

    /** The number of sub-ticks the simulator will run in each robot period. */
    private static int SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;

    public static int getSimulationSubTicksIn1Period() {
        return SIMULATION_SUB_TICKS_IN_1_PERIOD;
    }
    /** The period length of each sub-tick, in seconds. */
    private static Time SIMULATION_DT = Seconds.of(TimedRobot.kDefaultPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD);

    public static Time getSimulationDt() {
        return SIMULATION_DT;
    }

    /**
     *
     *
     * <h2>Returns the score of the specified team.</h2>
     *
     * @param isBlue The team to return the score of as a bool.
     * @return The score of the specified team.
     */
    public int getScore(boolean isBlue) {
        return isBlue ? blueScore : redScore;
    }

    /**
     *
     *
     * <h2>Returns the score of the specified team.</h2>
     *
     * @param allianceColor The team to return the score of as a Alliance enum.
     * @return The score of the specified team.
     */
    public int getScore(Alliance allianceColor) {
        return getScore(allianceColor == Alliance.Blue);
    }

    /**
     *
     *
     * <h2>Adds to the score of the specified team</h2>
     *
     * @param isBlue Wether to add to the blue or red team score.
     * @param toAdd How many points to add.
     */
    public void addToScore(boolean isBlue, int toAdd) {
        if (isBlue) blueScore += toAdd;
        else redScore += toAdd;
        addValueToMatchBreakdown(isBlue, DriverStation.isAutonomous() ? "Auto/AutoScore" : "TeleopScore", toAdd);
    }

    /**
     *
     *
     * <h2>Overrides the Timing Configurations of the Simulations.</h2>
     *
     * <h4>If Using <a href='https://github.com/Mechanical-Advantage/AdvantageKit'>Advantage-Kit</a>: DO NOT CHANGE THE
     * DEFAULT TIMINGS</h4>
     *
     * <p>Changes apply to every instance of {@link SimulatedArena}.
     *
     * <p>The new configuration will take effect the next time {@link SimulatedArena#simulationPeriodic()} is called on
     * an instance.
     *
     * <p>It is recommended to call this method before the first call to {@link SimulatedArena#simulationPeriodic()} of
     * any instance.
     *
     * <p>It is also recommended to keep the simulation frequency above 200 Hz for accurate simulation results.
     *
     * @param robotPeriod the time between two calls of {@link #simulationPeriodic()}, usually obtained from
     *     {@link TimedRobot#getPeriod()}
     * @param simulationSubTicksPerPeriod the number of Iterations, or {@link #simulationSubTick(int)} that the
     *     simulation runs per each call to {@link #simulationPeriodic()}
     */
    public static synchronized void overrideSimulationTimings(Time robotPeriod, int simulationSubTicksPerPeriod) {
        SIMULATION_SUB_TICKS_IN_1_PERIOD = simulationSubTicksPerPeriod;
        SIMULATION_DT = robotPeriod.div(SIMULATION_SUB_TICKS_IN_1_PERIOD);
    }

    protected final World<Body> physicsWorld;
    protected final Set<AbstractDriveTrainSimulation> driveTrainSimulations;

    protected final Set<GamePiece> gamePieces;
    protected final List<Simulatable> customSimulations;

    private final List<IntakeSimulation> intakeSimulations;

    /**
     *
     *
     * <h2>Constructs a new simulation arena with the specified field map of obstacles.</h2>
     *
     * <p>This constructor initializes a physics world with zero gravity and adds the provided obstacles to the world.
     *
     * <p>It also sets up the collections for drivetrain simulations, game pieces, projectiles, and intake simulations.
     *
     * @param obstaclesMap the season-specific field map containing the layout of obstacles for the simulation
     */
    protected SimulatedArena(FieldMap obstaclesMap) {
        this.physicsWorld = new World<>();
        this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
        for (Body obstacle : obstaclesMap.obstacles) this.physicsWorld.addBody(obstacle);
        this.driveTrainSimulations = new HashSet<>();
        customSimulations = new ArrayList<>();
        this.gamePieces = new HashSet<>();
        this.intakeSimulations = new ArrayList<>();
        setupValueForMatchBreakdown("TotalScore");
        setupValueForMatchBreakdown("TeleopScore");
        setupValueForMatchBreakdown("Auto/AutoScore");
        resetFieldPublisher.set(false);
    }

    /**
     *
     *
     * <h2>Represents a custom simulation to be updated during each simulation sub-tick.</h2>
     *
     * <p>This allows you to register custom actions that will be executed at a high frequency during each simulation
     * sub-tick. This is useful for tasks that need to be updated multiple times per simulation cycle.
     *
     * <p>Examples of how this method is used:
     *
     * <ul>
     *   <li>Pulling encoder values for high-frequency odometry updates.
     *   <li>Adding custom simulation objects or handling events in the simulated arena.
     * </ul>
     */
    public interface Simulatable {
        /**
         * Called in {@link #simulationSubTick(int)}.
         *
         * @param subTickNum the number of this sub-tick (counting from 0 in each robot period)
         */
        void simulationSubTick(int subTickNum);
    }

    /**
     *
     *
     * <h2>Registers a custom simulation.</h2>
     *
     * @param simulatable the custom simulation to register
     */
    public synchronized void addCustomSimulation(Simulatable simulatable) {
        this.customSimulations.add(simulatable);
    }

    /**
     *
     *
     * <h2>Registers an {@link IntakeSimulation}.</h2>
     *
     * <p><strong>NOTE:</strong> This method is automatically called in the constructor of {@link IntakeSimulation}, so
     * you don't need to call it manually.
     *
     * <p>The intake simulation should be bound to an {@link AbstractDriveTrainSimulation} and becomes part of its
     * collision space.
     *
     * <p>This method immediately starts the {@link org.ironmaple.simulation.IntakeSimulation.GamePieceContactListener},
     * which listens for contact between the intake and any game piece.
     *
     * @param intakeSimulation the intake simulation to be registered
     */
    protected synchronized void addIntakeSimulation(IntakeSimulation intakeSimulation) {
        this.intakeSimulations.add(intakeSimulation);
        this.physicsWorld.addContactListener(intakeSimulation.getGamePieceContactListener());
    }

    /**
     *
     *
     * <h2>Registers an {@link AbstractDriveTrainSimulation}.</h2>
     *
     * <p>The collision space of the drive train is immediately added to the simulation world.
     *
     * <p>Starting from the next call to {@link #simulationPeriodic()}, the
     * {@link AbstractDriveTrainSimulation#simulationSubTick()} method will be called during each sub-tick of the
     * simulator.
     *
     * @param driveTrainSimulation the drivetrain simulation to be registered
     */
    public synchronized void addDriveTrainSimulation(AbstractDriveTrainSimulation driveTrainSimulation) {
        this.physicsWorld.addBody(driveTrainSimulation);

        this.driveTrainSimulations.add(driveTrainSimulation);
    }

    /**
     *
     *
     * <h2>Registers a {@link GamePieceOnFieldSimulation} to the Simulation.</h2>
     *
     * <p>The collision space of the game piece is immediately added to the simulation world.
     *
     * <p>{@link IntakeSimulation}s will be able to interact with this game piece during the next call to
     * {@link SimulatedArena#simulationPeriodic()}.
     *
     * @param gamePiece the game piece to be registered in the simulation
     */
    public synchronized void addGamePiece(GamePieceOnFieldSimulation gamePiece) {
        this.physicsWorld.addBody(gamePiece);
        this.gamePieces.add(gamePiece);
    }

    /**
     *
     *
     * <h2>Tells the arena to start publishing the match breakdown data to network tables</h2>
     */
    public void enableBreakdownPublishing() {
        shouldPublishMatchBreakdown = true;
    }

    /**
     *
     *
     * <h2>Tells the arena to stop publishing the match breakdown data to network tables</h2>
     */
    public void disableBreakdownPublishing() {
        shouldPublishMatchBreakdown = false;
    }

    /**
     *
     *
     * <h2>Publishes the match breakdown data to network tables</h2>
     */
    protected void publishBreakdown() {

        for (String key : redScoringBreakdown.keySet()) {
            if (!redPublishers.containsKey(key))
                redPublishers.put(key, redTable.getDoubleTopic(key).publish());

            redPublishers.get(key).set(redScoringBreakdown.get(key));
        }
        for (String key : blueScoringBreakdown.keySet()) {
            if (!bluePublishers.containsKey(key))
                bluePublishers.put(key, blueTable.getDoubleTopic(key).publish());

            bluePublishers.get(key).set(blueScoringBreakdown.get(key));
        }

        // genericInfoTable.getDoubleTopic("currentMatchTime").publish().set(blueScore);
    }

    /**
     *
     *
     * <h2>replaces or adds a value to the match scoring breakdown published to network tables</h2>
     *
     * @param isBlueTeam Wether to add to the blue teams match breakdown or the red teams match breakdown
     * @param valueKey The name of the value to be added
     * @param value The value to be added
     */
    public void replaceValueInMatchBreakDown(boolean isBlueTeam, String valueKey, Double value) {
        if (isBlueTeam) blueScoringBreakdown.put(valueKey, value);
        else redScoringBreakdown.put(valueKey, value);
    }

    /**
     *
     *
     * <h2>Defaults a value to 0 and creates it in the match breakdown. This is useful on startup to make sure all match
     * breakdown values display before they are first updated</h2>
     *
     * @param valueKey The key to add to match breakdown
     */
    public void setupValueForMatchBreakdown(String valueKey) {
        replaceValueInMatchBreakDown(true, valueKey, 0);
        replaceValueInMatchBreakDown(false, valueKey, 0);
    }

    /**
     *
     *
     * <h2>replaces or adds a value to the match scoring breakdown published to network tables</h2>
     *
     * @param isBlueTeam Wether to add to the blue teams match breakdown or the red teams match breakdown
     * @param valueKey The name of the value to be added
     * @param value The value to be added
     */
    public void replaceValueInMatchBreakDown(boolean isBlueTeam, String valueKey, Integer value) {
        replaceValueInMatchBreakDown(isBlueTeam, valueKey, (double) value);
    }

    /**
     *
     *
     * <h2>Adds too a value in the scoring breakdown. If value does not already exist in the scoring breakdown it will
     * be defaulted to 0 and then added too
     *
     * @param isBlueTeam Wether to add to the blue teams match breakdown or the red teams match breakdown
     * @param ValueKey The name of the value to be added too
     * @param toAdd how much to be added to specified value
     */
    public void addValueToMatchBreakdown(boolean isBlueTeam, String ValueKey, Double toAdd) {
        if (isBlueTeam) {
            if (blueScoringBreakdown.get(ValueKey) == null) blueScoringBreakdown.put(ValueKey, toAdd);
            else blueScoringBreakdown.put(ValueKey, blueScoringBreakdown.get(ValueKey) + toAdd);
        } else {
            if (redScoringBreakdown.get(ValueKey) == null) redScoringBreakdown.put(ValueKey, toAdd);
            else redScoringBreakdown.put(ValueKey, redScoringBreakdown.get(ValueKey) + toAdd);
        }
    }

    /**
     *
     *
     * <h2>Adds too a value in the scoring breakdown. If value does not already exist in the scoring breakdown it will
     * be defaulted to 0 and then added too
     *
     * @param isBlueTeam Wether to add to the blue teams match breakdown or the red teams match breakdown
     * @param valueKey The name of the value to be added too
     * @param toAdd how much to be added to specified value
     */
    public void addValueToMatchBreakdown(boolean isBlueTeam, String valueKey, int toAdd) {
        addValueToMatchBreakdown(isBlueTeam, valueKey, (double) toAdd);
    }

    /**
     *
     *
     * <h2>Registers a {@link GamePieceProjectile} to the Simulation and Launches It.</h2>
     *
     * <p>Calls to {@link GamePieceProjectile#launch()}, which will launch the game piece immediately.
     *
     * @param gamePieceProjectile the projectile to be registered and launched in the simulation
     */
    public synchronized void addGamePieceProjectile(GamePieceProjectile gamePieceProjectile) {
        this.gamePieces.add(gamePieceProjectile);
        gamePieceProjectile.launch();
    }

    /**
     *
     *
     * <h2>Removes a {@link GamePieceOnFieldSimulation} from the Simulation.</h2>
     *
     * <p>Removes the game piece from the physics world and the simulation's game piece collection.
     *
     * @param gamePiece the game piece to be removed from the simulation
     * @return <code>true</code> if this set contained the specified element
     */
    public synchronized boolean removeGamePiece(GamePieceOnFieldSimulation gamePiece) {
        this.physicsWorld.removeBody(gamePiece);
        return this.gamePieces.remove(gamePiece);
    }

    public synchronized boolean removePiece(GamePiece toRemove) {
        if (toRemove.isGrounded()) {
            return removeGamePiece((GamePieceOnFieldSimulation) toRemove);
        }
        return removeProjectile((GamePieceProjectile) toRemove);
    }

    /**
     *
     *
     * <h2>Removes a {@link GamePieceProjectile} from the Simulation.</h2>
     *
     * <p>Removes the game piece projectile from the simulation.
     *
     * @param gamePieceLaunched the game piece projectile to be removed from the simulation
     * @return <code>true</code> if this set contained the specified element
     */
    public synchronized boolean removeProjectile(GamePieceProjectile gamePieceLaunched) {
        return this.gamePieces.remove(gamePieceLaunched);
    }

    /**
     *
     *
     * <h2>Removes All {@link GamePieceOnFieldSimulation} Objects from the Simulation.</h2>
     *
     * <p>This method clears all game pieces from the physics world and the simulation's game piece collection.
     */
    public synchronized void clearGamePieces() {
        for (GamePieceOnFieldSimulation gamePiece : this.gamePiecesOnField()) this.physicsWorld.removeBody(gamePiece);

        this.gamePieces.clear();
        this.blueScore = 0;
        this.redScore = 0;
    }

    /**
     *
     *
     * <h2>Shuts down the current SimulatedArena and stops all simulation so that simulatable objects may be added to a
     * new arena </h2>
     */
    public synchronized void shutDown() {
        this.physicsWorld.removeAllBodies();
    }

    /**
     *
     *
     * <h2>Update the simulation world.</h2>
     *
     * <p>This method should be called ONCE in {@link TimedRobot#simulationPeriodic()} (or <code>
     * LoggedRobot.simulationPeriodic()</code> if using <a
     * href='https://github.com/Mechanical-Advantage/AdvantageKit'>Advantage-Kit</a>)
     *
     * <p>If not configured through {@link SimulatedArena#overrideSimulationTimings(Time, int)} , the simulator will
     * iterate through 5 Sub-ticks by default.
     *
     * <p>The amount of CPU Time that the Dyn4j engine uses in displayed in <code>
     * SmartDashboard/MapleArenaSimulation/Dyn4jEngineCPUTimeMS</code>, usually performance is not a concern
     */
    public synchronized void simulationPeriodic() {
        /* obtain lock to the simulated arena class to block any calls to overrideTimings() */
        synchronized (SimulatedArena.class) {
            final long t0 = System.nanoTime();
            // move through a few sub-periods in each update
            for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++) simulationSubTick(i);

            matchClock += getSimulationDt().in(Units.Seconds);

            SmartDashboard.putNumber("MapleArenaSimulation/Dyn4jEngineCPUTimeMS", (System.nanoTime() - t0) / 1000000.0);

            if (resetFieldSubscriber.get()) {
                SimulatedArena.getInstance().resetFieldForAuto();
                resetFieldPublisher.set(false);
                matchClock = 0;
            }
        }
    }

    /**
     *
     *
     * <h2>Processes a Single Simulation Sub-Tick.</h2>
     *
     * <p>This method performs the actions for each sub-tick of the simulation, including:
     *
     * <ul>
     *   <li>Updating all registered {@link AbstractDriveTrainSimulation} objects.
     *   <li>Updating all {@link GamePieceProjectile} objects in the simulation.
     *   <li>Stepping the physics world with the specified sub-tick duration.
     *   <li>Removing any game pieces as detected by the {@link IntakeSimulation} objects.
     *   <li>Executing any additional sub-tick actions registered via
     *       {@link SimulatedArena#addCustomSimulation(Simulatable)} .
     * </ul>
     */
    protected void simulationSubTick(int subTickNum) {
        SimulatedBattery.simulationSubTick();
        driveTrainSimulations.forEach(AbstractDriveTrainSimulation::simulationSubTick);

        GamePieceProjectile.updateGamePieceProjectiles(this, this.gamePieceLaunched());

        this.physicsWorld.step(1, SIMULATION_DT.in(Seconds));

        intakeSimulations.forEach(intake -> intake.removeObtainedGamePieces(this));
        customSimulations.forEach(sim -> sim.simulationSubTick(subTickNum));

        replaceValueInMatchBreakDown(true, "TotalScore", blueScore);
        replaceValueInMatchBreakDown(false, "TotalScore", redScore);

        if (shouldPublishMatchBreakdown) {
            publishBreakdown();
            matchClockPublisher.set(matchClock);
        }
    }

    /**
     *
     *
     * <h2>Returns a list of all grounded pieces on the field</h2>
     *
     * @return all grounded (aka not projectile) pieces on the field as a set of GamePieceOnFieldSimulation objects
     */
    public synchronized Set<GamePieceOnFieldSimulation> gamePiecesOnField() {
        Set<GamePieceOnFieldSimulation> returnList = new HashSet<GamePieceOnFieldSimulation>();
        for (GamePiece gamePiece : this.gamePieces) {
            if (gamePiece.isGrounded()) {
                returnList.add((GamePieceOnFieldSimulation) gamePiece);
            }
        }

        return returnList;
    }

    /**
     *
     *
     * <h2>Returns a list of all projectile pieces on the field</h2>
     *
     * @return all projectile pieces on the field as a set of GamePieceProjectile objects
     */
    public synchronized Set<GamePieceProjectile> gamePieceLaunched() {
        Set<GamePieceProjectile> returnList = new HashSet<GamePieceProjectile>();
        for (GamePiece gamePiece : this.gamePieces) {
            if (!gamePiece.isGrounded()) {
                returnList.add((GamePieceProjectile) gamePiece);
            }
        }

        return returnList;
    }

    /**
     *
     *
     * <h2>Obtains the 3D Poses of a Specific Type of Game Piece.</h2>
     *
     * <p>This method is used to visualize the positions of game pieces
     *
     * <p>Also, if you have a game-piece detection vision system <strong>(wow!)</strong>, this is the how you can
     * simulate it.
     *
     * <p>Both {@link GamePieceOnFieldSimulation} and {@link GamePieceProjectile} of the specified type will be
     * included.
     *
     * <ul>
     *   <li>The type is determined in the constructor of {@link GamePieceOnFieldSimulation}.
     *   <li>For example, {@link org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField} has the
     *       type "Note".
     * </ul>
     *
     * @param type the type of game piece, as determined by the constructor of {@link GamePieceOnFieldSimulation}
     * @return a {@link List} of {@link Pose3d} objects representing the 3D positions of the game pieces
     */
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        final List<Pose3d> gamePiecesPoses = new ArrayList<>();
        for (GamePiece gamePiece : gamePieces)
            if (Objects.equals(gamePiece.getType(), type)) gamePiecesPoses.add(gamePiece.getPose3d());

        return gamePiecesPoses;
    }

    /**
     *
     *
     * <h2>Obtains the 3D Poses of a Specific Type of Game Piece as an array.</h2>
     *
     * @see #getGamePiecesPosesByType(String)
     */
    public synchronized Pose3d[] getGamePiecesArrayByType(String type) {
        return getGamePiecesPosesByType(type).toArray(Pose3d[]::new);
    }

    /**
     *
     *
     * <h2>Returns all game pieces on the field of the specified type as a list
     *
     * @param type The string type to be selected.
     * @return The game pieces as a list of {@link GamePiece}
     */
    public synchronized List<GamePiece> getGamePiecesByType(String type) {
        final List<GamePiece> gamePiecesPoses = new ArrayList<>(this.gamePieces);
        gamePiecesPoses.stream().filter(gamePiece -> !Objects.equals(gamePiece.getType(), type));
        return gamePiecesPoses;
    }

    /**
     *
     *
     * <h2>Resets the Field for Autonomous Mode.</h2>
     *
     * <p>This method clears all current game pieces from the field and places new game pieces in their starting
     * positions for the autonomous mode.
     */
    public synchronized void resetFieldForAuto() {
        clearGamePieces();
        matchClock = 0;
        placeGamePiecesOnField();
    }

    /**
     *
     *
     * <h2>Places Game Pieces on the Field for Autonomous Mode.</h2>
     *
     * <p>This method sets up the game pieces on the field, typically in their starting positions for autonomous mode.
     *
     * <p>It should be implemented differently for each season-specific subclass of {@link SimulatedArena} to reflect
     * the unique game piece placements for that season's game.
     */
    public abstract void placeGamePiecesOnField();

    /**
     *
     *
     * <h1>Represents an Abstract Field Map</h1>
     *
     * <p>Stores the layout of obstacles and game pieces.
     *
     * <p>For each season-specific subclass of {@link SimulatedArena}, there should be a corresponding subclass of this
     * class to store the field map for that specific season's game.
     */
    public abstract static class FieldMap {
        private final List<Body> obstacles = new ArrayList<>();

        protected void addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
            addCustomObstacle(
                    Geometry.createSegment(
                            GeometryConvertor.toDyn4jVector2(startingPoint),
                            GeometryConvertor.toDyn4jVector2(endingPoint)),
                    new Pose2d());
        }

        protected void addRectangularObstacle(double width, double height, Pose2d absolutePositionOnField) {
            addCustomObstacle(Geometry.createRectangle(width, height), absolutePositionOnField);
        }

        protected void addCustomObstacle(Convex shape, Pose2d absolutePositionOnField) {
            final Body obstacle = createObstacle(shape);

            obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(absolutePositionOnField));

            obstacles.add(obstacle);
        }

        private static Body createObstacle(Convex shape) {
            final Body obstacle = new Body();
            obstacle.setMass(MassType.INFINITE);
            final BodyFixture fixture = obstacle.addFixture(shape);
            fixture.setFriction(0.6);
            fixture.setRestitution(0.3);
            return obstacle;
        }
    }
}
