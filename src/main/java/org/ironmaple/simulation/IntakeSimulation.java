package org.ironmaple.simulation;

import org.dyn4j.collision.CollisionBody;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListener;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import java.util.ArrayDeque;
import java.util.Objects;
import java.util.Queue;

/**
 * <h2>Simulates an Intake Mechanism on the Robot.</h2>
 *
 * <h3><a href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/docs/5_SIMULATING_INTAKE.md'>Online Documentation</a></h3>
 *
 * <p>The intake is a 2D component attached to one side of the robot's chassis. It is rectangular in shape and extends from the robot when activated.</p>
 * <p>The intake can be turned on through {@link #startIntake()}, which causes it to extend, expanding the collision space of the robot's chassis. When turned off via {@link #stopIntake()}, the intake retracts.</p>
 * <p>The intake can "collect" {@link GamePieceOnFieldSimulation} instances from the field, removing them and incrementing the {@link #gamePiecesInIntakeCount}.</p>
 *
 * <p>A game piece is collected if the following conditions are met:</p>
 * <ul>
 *   <li>1. The type of the game piece ({@link org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation#type}) matches {@link #targetedGamePieceType}.</li>
 *   <li>2. The {@link org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation} is in contact with the intake (and not other parts of the robot).</li>
 *   <li>3. The intake is turned on via {@link #startIntake()}.</li>
 *   <li>4. The number of game pieces in the intake ({@link #gamePiecesInIntakeCount}) is less than {@link #capacity}.</li>
 * </ul>
 *
 * <p><strong>Note:</strong> This class simulates an idealized "touch it, get it" intake and does not model the actual functioning of an intake mechanism.</p>
 * */
public abstract class IntakeSimulation extends BodyFixture {
    private final int capacity;
    protected int gamePiecesInIntakeCount;
    private boolean intakeRunning;

    private final Queue<GamePieceOnFieldSimulation> gamePiecesToRemove;
    private final AbstractDriveTrainSimulation driveTrainSimulation;
    private final String targetedGamePieceType;

    public enum IntakeSide {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }

    /**
     * <h2>Creates an Intake Simulation that Tightly Attaches to One Side of the Chassis.</h2>
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param width the width of the intake, in meters
     * @param side the side of the chassis where the intake is attached
     * @param capacity the maximum number of game pieces that the intake can hold
     * */
    public IntakeSimulation(String targetedGamePieceType, AbstractDriveTrainSimulation driveTrainSimulation, double width, IntakeSide side, int capacity) {
        this(
                targetedGamePieceType,
                driveTrainSimulation,
                width,
                0.02,
                side,
                capacity
        );
    }

    /**
     * <h2>Creates an Intake Simulation that Extends Out of the Chassis Frame.</h2>
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param width the valid width of the intake, in meters
     * @param lengthExtended the length the intake extends out from the chassis when activated, in meters
     * @param side the side of the chassis where the intake is attached
     * @param capacity the maximum number of game pieces that the intake can hold
     * */
    public IntakeSimulation(String targetedGamePieceType, AbstractDriveTrainSimulation driveTrainSimulation, double width, double lengthExtended, IntakeSide side, int capacity) {
        this(
                targetedGamePieceType,
                driveTrainSimulation,
                getIntakeRectangle(driveTrainSimulation, width, lengthExtended, side),
                capacity
        );
    }

    private static Rectangle getIntakeRectangle(AbstractDriveTrainSimulation driveTrainSimulation, double width, double lengthExtended, IntakeSide side) {
        final Rectangle intakeRectangle = new Rectangle(width, lengthExtended);
        intakeRectangle.rotate(switch (side) {
            case LEFT, RIGHT -> 0;
            case FRONT, BACK -> Math.toRadians(90);
        });
        final double distanceTransformed = lengthExtended / 2 - 0.01;
        intakeRectangle.translate(switch (side) {
            case LEFT -> new Vector2(0, driveTrainSimulation.profile.width / 2 + distanceTransformed);
            case RIGHT -> new Vector2(0, -driveTrainSimulation.profile.width / 2 - distanceTransformed);
            case FRONT -> new Vector2(driveTrainSimulation.profile.length / 2 + distanceTransformed, 0);
            case BACK -> new Vector2(-driveTrainSimulation.profile.length / 2 - distanceTransformed / 2, 0);
        });

        return intakeRectangle;
    };

    /**
     * <h2>Creates an Intake Simulation with a Specific Shape.</h2>
     *
     * <p>This constructor initializes an intake with a custom shape that is used when the intake is fully extended.</p>
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param shape the shape of the intake when fully extended, represented as a {@link Convex} object
     * @param capacity the maximum number of game pieces that the intake can hold
     * */
    public IntakeSimulation(String targetedGamePieceType, AbstractDriveTrainSimulation driveTrainSimulation, Convex shape, int capacity) {
        super(shape);

        this.targetedGamePieceType = targetedGamePieceType;
        this.gamePiecesInIntakeCount = 0;

        if (capacity > 100)
            throw new IllegalArgumentException("capacity too large, max is 100");
        this.capacity = capacity;

        this.gamePiecesToRemove = new ArrayDeque<>(capacity);

        this.intakeRunning = false;
        this.driveTrainSimulation = driveTrainSimulation;
    }

    /**
     * <h2>Turns the Intake On.</h2>
     *
     * <p>Extends the intake out from the chassis, making it part of the chassis's collision space.</p>
     *
     * <p>Once activated, the intake is considered running and will listen for contact with {@link GamePieceOnFieldSimulation} instances, allowing it to collect game pieces.</p>
     * */
    protected void startIntake() {
        if (intakeRunning) return;

        driveTrainSimulation.addFixture(this);
        this.intakeRunning = true;
    }

    /**
     * <h2>Turns the Intake Off.</h2>
     *
     * <p>Retracts the intake into the chassis, removing it from the chassis's collision space.</p>
     *
     * <p>Once turned off, the intake will no longer listen for or respond to contacts with {@link GamePieceOnFieldSimulation} instances.</p>
     * */
    protected void stopIntake() {
        if (!intakeRunning) return;

        driveTrainSimulation.removeFixture(this);
        this.intakeRunning = false;
    }

    /**
     * <h2>The {@link ContactListener} for the Intake Simulation.</h2>
     *
     * <p>This class can be added to the simulation world to detect and manage contacts between the intake and {@link GamePieceOnFieldSimulation} instances of the type {@link #targetedGamePieceType}.</p>
     *
     * <p>If contact is detected and the intake is running, the {@link GamePieceOnFieldSimulation} will be marked for removal from the field.</p>
     * */
    public final class GamePieceContactListener implements ContactListener<Body> {
        @Override
        public void begin(ContactCollisionData collision, Contact contact) {
            if (!intakeRunning) return;
            if (gamePiecesInIntakeCount >= capacity) return;

            final CollisionBody<?> collisionBody1 = collision.getBody1(), collisionBody2 = collision.getBody2();
            final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

            if (collisionBody1 instanceof GamePieceOnFieldSimulation gamePiece
                    && Objects.equals(gamePiece.type, targetedGamePieceType)
                    && fixture2 == IntakeSimulation.this)
                flagGamePieceForRemoval(gamePiece);

            else if (collisionBody2 instanceof GamePieceOnFieldSimulation gamePiece
                    && Objects.equals(gamePiece.type, targetedGamePieceType)
                    && fixture1 == IntakeSimulation.this)
                flagGamePieceForRemoval(gamePiece);
        }

        private void flagGamePieceForRemoval(GamePieceOnFieldSimulation gamePiece) {
            gamePiecesToRemove.add(gamePiece);
            gamePiecesInIntakeCount++;
        }

        /* functions not used */
        @Override public void persist(ContactCollisionData collision, Contact oldContact, Contact newContact) {}
        @Override public void end(ContactCollisionData collision, Contact contact) {}
        @Override public void destroyed(ContactCollisionData collision, Contact contact) {}
        @Override public void collision(ContactCollisionData collision) {}
        @Override public void preSolve(ContactCollisionData collision, Contact contact) {}
        @Override public void postSolve(ContactCollisionData collision, SolvedContact contact) {}
    }

    /**
     * <h2>Obtains a New Instance of the {@link GamePieceContactListener} for This Intake.</h2>
     *
     * @return a new {@link GamePieceContactListener} for this intake
     * */
    public GamePieceContactListener getGamePieceContactListener() {
        return new GamePieceContactListener();
    }

    /**
     * <h2>Obtains the {@link GamePieceOnFieldSimulation} Instances to Be Removed from the Field.</h2>
     *
     * <p>This method is called from {@link SimulatedArena#simulationPeriodic()} to retrieve game pieces that have been marked for removal.</p>
     * <p>Game pieces are marked for removal if they have come into contact with the intake during the last {@link SimulatedArena#getSimulationSubTicksIn1Period()} sub-ticks. These game pieces should be removed from the field to reflect their interaction with the intake.</p>
     *
     * @return a {@link Queue} of game pieces to be removed from the field
     * */
    public Queue<GamePieceOnFieldSimulation> getGamePiecesToRemove() {
        return gamePiecesToRemove;
    }

    /**
     * <h2>Clears the Game Pieces Marked for Removal.</h2>
     *
     * <p>This method is called from {@link SimulatedArena#simulationPeriodic()} after the game pieces marked for removal have been processed and removed from the field.</p>
     *
     * <p>It clears the queue of game pieces marked for removal</p>
     * */
    public void clearGamePiecesToRemoveQueue() {
        gamePiecesToRemove.clear();
    }

    public void register() {
        register(SimulatedArena.getInstance());
    }

    public void register(SimulatedArena arena) {
        arena.addIntakeSimulation(this);
    }
}
