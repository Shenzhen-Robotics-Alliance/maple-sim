package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.ref.WeakReference;
import java.util.*;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h1>Abstract Simulation World</h1>
 *
 * <h2>The heart of the simulator.</h2>
 *
 * <p>This class cannot be instantiated directly; it must be created as a specific season's arena.
 *
 * <p>The default instance can be obtained using the {@link #getInstance()} method.
 *
 * <p>Simulates all interactions within the arena field.
 *
 * <h2>The following objects can be added to the simulation world and will interact with each other:
 * </h2>
 *
 * <ul>
 *   <li>{@link AbstractDriveTrainSimulation}: Represents abstract drivetrain simulations with
 *       collision detection.
 *   <li>{@link GamePieceOnFieldSimulation}: Represents abstract game pieces with collision
 *       detection.
 *   <li>{@link IntakeSimulation}: Represents an intake simulation that responds to contact with
 *       {@link GamePieceOnFieldSimulation}.
 * </ul>
 */
public abstract class SimulatedArena {
  private static SimulatedArena instance = null;
  /**
   *
   *
   * <h2>Gets/Creates the Default Simulation World</h2>
   *
   * <p>Multiple instances of {@link SimulatedArena} can exist elsewhere.
   *
   * @return the main simulation arena instance
   */
  public static SimulatedArena getInstance() {
    if (instance == null) instance = new Arena2024Crescendo();
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
    instance = newInstance;
  }

  /** The number of sub-ticks the simulator will run in each robot period. */
  private static int SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;

  public static int getSimulationSubTicksIn1Period() {
    return SIMULATION_SUB_TICKS_IN_1_PERIOD;
  }
  /** The period length of each sub-tick, in seconds. */
  private static double SIMULATION_DT =
      TimedRobot.kDefaultPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD;

  public static double getSimulationDt() {
    return SIMULATION_DT;
  }

  /**
   *
   *
   * <h2>Overrides the Timing Configurations of the Simulations.</h2>
   *
   * <h4>If Using <a href='https://github.com/Mechanical-Advantage/AdvantageKit'>Advantage-Kit</a>:
   * DO NOT CHANGE THE DEFAULT TIMINGS</h4>
   *
   * <p>Changes apply to every instance of {@link SimulatedArena}.
   *
   * <p>The new configuration will take effect the next time {@link
   * SimulatedArena#simulationPeriodic()} is called on an instance.
   *
   * <p>It is recommended to call this method before the first call to {@link
   * SimulatedArena#simulationPeriodic()} of any instance.
   *
   * <p>It is also recommended to keep the simulation frequency above 200 Hz for accurate simulation
   * results.
   *
   * @param robotPeriodSeconds the time between two calls of {@link #simulationPeriodic()}, usually
   *     obtained from {@link TimedRobot#getPeriod()}
   * @param simulationSubTicksPerPeriod the number of Iterations, or {@link #simulationSubTick()}
   *     that the simulation runs per each call to {@link #simulationPeriodic()}
   */
  public static void overrideSimulationTimings(
      double robotPeriodSeconds, int simulationSubTicksPerPeriod) {
    SIMULATION_SUB_TICKS_IN_1_PERIOD = simulationSubTicksPerPeriod;
    SIMULATION_DT = robotPeriodSeconds / SIMULATION_SUB_TICKS_IN_1_PERIOD;
  }

  protected final World<Body> physicsWorld;
  protected final Set<AbstractDriveTrainSimulation> driveTrainSimulations;
  protected final Set<GamePieceOnFieldSimulation> gamePieces;
  protected final Set<GamePieceProjectile> gamePieceProjectile;
  protected final List<Runnable> simulationSubTickActions;
  protected final List<WeakReference<MapleMotorSim>> motors;
  private final List<IntakeSimulation> intakeSimulations;

  /**
   *
   *
   * <h2>Constructs a new simulation arena with the specified field map of obstacles.</h2>
   *
   * <p>This constructor initializes a physics world with zero gravity and adds the provided
   * obstacles to the world.
   *
   * <p>It also sets up the collections for drivetrain simulations, game pieces, projectiles, and
   * intake simulations.
   *
   * @param obstaclesMap the season-specific field map containing the layout of obstacles for the
   *     simulation
   */
  protected SimulatedArena(FieldMap obstaclesMap) {
    this.physicsWorld = new World<>();
    this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
    for (Body obstacle : obstaclesMap.obstacles) this.physicsWorld.addBody(obstacle);
    this.driveTrainSimulations = new HashSet<>();
    simulationSubTickActions = new ArrayList<>();
    this.gamePieces = new HashSet<>();
    this.gamePieceProjectile = new HashSet<>();
    this.intakeSimulations = new ArrayList<>();
    motors = new ArrayList<>();
  }

  /**
   *
   *
   * <h2>Registers a runnable action to be executed during each simulation sub-tick.</h2>
   *
   * <p><strong>FOR TESTING ONLY: This method will be removed in the final release.</strong>
   *
   * @param action the {@link Runnable} action to be executed in each simulation sub-tick
   */
  @Deprecated
  public void addSimulationSubTickAction(Runnable action) {
    this.simulationSubTickActions.add(action);
  }

  /**
   *
   *
   * <h2>Registers an {@link IntakeSimulation}.</h2>
   *
   * <p><strong>NOTE:</strong> This method is automatically called in the constructor of {@link
   * IntakeSimulation}, so you don't need to call it manually.
   *
   * <p>The intake simulation should be bound to an {@link AbstractDriveTrainSimulation} and becomes
   * part of its collision space.
   *
   * <p>This method immediately starts the {@link
   * org.ironmaple.simulation.IntakeSimulation.GamePieceContactListener}, which listens for contact
   * between the intake and any game piece.
   *
   * @param intakeSimulation the intake simulation to be registered
   */
  protected void addIntakeSimulation(IntakeSimulation intakeSimulation) {
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
   * <p>Starting from the next call to {@link #simulationPeriodic()}, the {@link
   * AbstractDriveTrainSimulation#simulationSubTick()} method will be called during each sub-tick of
   * the simulator.
   *
   * @param driveTrainSimulation the drivetrain simulation to be registered
   */
  public void addDriveTrainSimulation(AbstractDriveTrainSimulation driveTrainSimulation) {
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
   * <p>{@link IntakeSimulation}s will be able to interact with this game piece during the next call
   * to {@link SimulatedArena#simulationPeriodic()}.
   *
   * @param gamePiece the game piece to be registered in the simulation
   */
  public void addGamePiece(GamePieceOnFieldSimulation gamePiece) {
    this.physicsWorld.addBody(gamePiece);
    this.gamePieces.add(gamePiece);
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
  public void addGamePieceProjectile(GamePieceProjectile gamePieceProjectile) {
    this.gamePieceProjectile.add(gamePieceProjectile);
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
   */
  public void removeGamePiece(GamePieceOnFieldSimulation gamePiece) {
    this.physicsWorld.removeBody(gamePiece);
    this.gamePieces.remove(gamePiece);
  }

  /**
   *
   *
   * <h2>Removes All {@link GamePieceOnFieldSimulation} Objects from the Simulation.</h2>
   *
   * <p>This method clears all game pieces from the physics world and the simulation's game piece
   * collection.
   */
  public void clearGamePieces() {
    for (GamePieceOnFieldSimulation gamePiece : this.gamePieces)
      this.physicsWorld.removeBody(gamePiece);
    this.gamePieces.clear();
  }

  public void addMotor(MapleMotorSim motor) {
    motors.add(new WeakReference<>(motor));
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
   * <p>If not configured through {@link SimulatedArena#overrideSimulationTimings(double
   * robotPeriodSeconds, int simulationSubTicksPerPeriod)}, the simulator will iterate through 5
   * Sub-ticks by default.
   *
   * <p>The amount of CPU Time that the Dyn4j engine uses in displayed in <code>
   * SmartDashboard/MapleArenaSimulation/Dyn4jEngineCPUTimeMS</code>, usually performance is not a
   * concern
   */
  public void simulationPeriodic() {
    final long t0 = System.nanoTime();
    competitionPeriodic();
    // move through a few sub-periods in each update
    for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++) simulationSubTick();

    SmartDashboard.putNumber(
        "MapleArenaSimulation/Dyn4jEngineCPUTimeMS", (System.nanoTime() - t0) / 1000000.0);
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
   *   <li>Executing any additional sub-tick actions registered via {@link
   *       SimulatedArena#addSimulationSubTickAction(Runnable)}.
   * </ul>
   */
  private void simulationSubTick() {
    ArrayList<Double> motorCurrents = new ArrayList<>();
    for (var motor : motors) {
      MapleMotorSim motorRef = motor.get();
      if (motorRef != null) {
        motorRef.update();
      }
    }
    double vin =
        BatterySim.calculateLoadedBatteryVoltage(
            12.2, 0.015, motorCurrents.stream().mapToDouble(Double::doubleValue).toArray());
    RoboRioSim.setVInVoltage(vin);

    for (AbstractDriveTrainSimulation driveTrainSimulation : driveTrainSimulations)
      driveTrainSimulation.simulationSubTick();

    GamePieceProjectile.updateGamePieceProjectiles(this, this.gamePieceProjectile);

    this.physicsWorld.step(1, SIMULATION_DT);

    for (IntakeSimulation intakeSimulation : intakeSimulations)
      while (!intakeSimulation.getGamePiecesToRemove().isEmpty())
        removeGamePiece(intakeSimulation.getGamePiecesToRemove().poll());

    for (Runnable runnable : simulationSubTickActions) runnable.run();
  }

  /**
   *
   *
   * <h2>Obtains the 3D Poses of a Specific Type of Game Piece.</h2>
   *
   * <p>This method is used to visualize the positions of game pieces
   *
   * <p>Also, if you have a game-piece detection vision system <strong>(wow!)</strong>, this is the
   * how you can simulate it.
   *
   * <p>Both {@link GamePieceOnFieldSimulation} and {@link GamePieceProjectile} of the specified
   * type will be included.
   *
   * <ul>
   *   <li>The type is determined in the constructor of {@link GamePieceOnFieldSimulation}.
   *   <li>For example, {@link
   *       org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField} has the type
   *       "Note".
   * </ul>
   *
   * @param type the type of game piece, as determined by the constructor of {@link
   *     GamePieceOnFieldSimulation}
   * @return a {@link List} of {@link Pose3d} objects representing the 3D positions of the game
   *     pieces
   */
  public List<Pose3d> getGamePiecesByType(String type) {
    final List<Pose3d> gamePiecesPoses = new ArrayList<>();
    for (GamePieceOnFieldSimulation gamePiece : gamePieces)
      if (Objects.equals(gamePiece.type, type)) gamePiecesPoses.add(gamePiece.getPose3d());

    for (GamePieceProjectile gamePiece : gamePieceProjectile)
      if (Objects.equals(gamePiece.gamePieceType, type)) gamePiecesPoses.add(gamePiece.getPose3d());

    return gamePiecesPoses;
  }

  /**
   *
   *
   * <h2>Resets the Field for Autonomous Mode.</h2>
   *
   * <p>This method clears all current game pieces from the field and places new game pieces in
   * their starting positions for the autonomous mode.
   */
  public void resetFieldForAuto() {
    clearGamePieces();
    placeGamePiecesOnField();
  }

  /**
   *
   *
   * <h2>Places Game Pieces on the Field for Autonomous Mode.</h2>
   *
   * <p>This method sets up the game pieces on the field, typically in their starting positions for
   * autonomous mode.
   *
   * <p>It should be implemented differently for each season-specific subclass of {@link
   * SimulatedArena} to reflect the unique game piece placements for that season's game.
   */
  public abstract void placeGamePiecesOnField();

  /**
   *
   *
   * <h2>Season-Specific Actions to Execute in {@link SimulatedArena#simulationPeriodic()}.</h2>
   *
   * <p>This method defines season-specific tasks to be executed during the {@link
   * SimulatedArena#simulationPeriodic()} method.
   *
   * <p>For example:
   *
   * <ul>
   *   <li>Updating the score counts.
   *   <li>Simulating human player activities.
   * </ul>
   *
   * <p>This method should be implemented in the season-specific subclass of {@link SimulatedArena}
   * to reflect the unique aspects of that season's game.
   */
  public abstract void competitionPeriodic();

  /**
   *
   *
   * <h1>Represents an Abstract Field Map</h1>
   *
   * <p>Stores the layout of obstacles and game pieces.
   *
   * <p>For each season-specific subclass of {@link SimulatedArena}, there should be a corresponding
   * subclass of this class to store the field map for that specific season's game.
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

    protected void addRectangularObstacle(
        double width, double height, Pose2d absolutePositionOnField) {
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
