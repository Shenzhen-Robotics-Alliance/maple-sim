package org.ironmaple.simulation;

import java.util.List;

import org.dyn4j.collision.CollisionBody;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.Convex;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListener;
import org.ironmaple.simulation.GamePiece.GamePieceCollisionBody;
import org.ironmaple.simulation.GamePiece.GamePieceVariant;
import org.ironmaple.simulation.drivesims.DriveTrainSimulation;

/**
 *
 *
 * <h2>Simulates an Intake Mechanism on the Robot.</h2>
 *
 * <h3><a
 * href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/docs/5_SIMULATING_INTAKE.md'>Online
 * Documentation</a></h3>
 *
 * <p>The intake is a 2D component attached to one side of the robot's chassis. It is rectangular in
 * shape and extends from the robot when activated.
 *
 * <p>The intake can be turned on through {@link #startIntake()}, which causes it to extend,
 * expanding the collision space of the robot's chassis. When turned off via {@link #stopIntake()},
 * the intake retracts.
 *
 * <p>The intake can "collect" {@link GamePieceOnFieldSimulation} instances from the field, removing
 * them and incrementing the {@link #gamePiecesInIntakeCount}.
 *
 * <p>A game piece is collected if the following conditions are met:
 *
 * <ul>
 *   <li>1. The type of the game piece ({@link
 *       org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation#type}) matches {@link
 *       #targetedGamePieceType}.
 *   <li>2. The {@link org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation} is in contact
 *       with the intake (and not other parts of the robot).
 *   <li>3. The intake is turned on via {@link #startIntake()}.
 *   <li>4. The number of game pieces in the intake ({@link #gamePiecesInIntakeCount}) is less than
 *       {@link #capacity}.
 * </ul>
 *
 * <p><strong>Note:</strong> This class simulates an idealized "touch it, get it" intake and does
 * not model the actual functioning of an intake mechanism.
 */
public class IntakeSimulation extends BodyFixture {
    private boolean intakeRunning = false;

    private final DriveTrainSimulation driveTrainSimulation;
    private final List<GamePieceVariant> acceptedGamePieceVariants;
    private final GamePieceStorage gamePieceStorage;
    private final IntakeBehavior intakeBehavior;

    public enum IntakeBehavior {
        ADD_HIGHEST_AVAILABLE,
        ADD_LOWEST_AVAILABLE,
        ADD_HIGHEST_DISPLACE,
        ADD_LOWEST_DISPLACE,
        ADD_RANDOM,
    }

    /**
     *
     *
     * <h2>Creates an Intake Simulation with a Specific Shape.</h2>
     *
     * <p>This constructor initializes an intake with a custom shape that is used when the intake is
     * fully extended.
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param shape the shape of the intake when fully extended, represented as a {@link Convex}
     *     object
     * @param capacity the maximum number of game pieces that the intake can hold
     */
    IntakeSimulation(
            DriveTrainSimulation driveTrainSimulation,
            GamePieceStorage gamePieceStorage,
            Convex shape,
            IntakeBehavior intakeBehavior,
            GamePieceVariant... acceptedGamePieceVariants) {
        super(shape);

        this.acceptedGamePieceVariants = List.of(acceptedGamePieceVariants);
        this.driveTrainSimulation = driveTrainSimulation;
        this.gamePieceStorage = gamePieceStorage;
        this.intakeBehavior = intakeBehavior;
    }

    /**
     *
     *
     * <h2>Turns the Intake On.</h2>
     *
     * <p>Extends the intake out from the chassis, making it part of the chassis's collision space.
     *
     * <p>Once activated, the intake is considered running and will listen for contact with {@link
     * GamePieceOnFieldSimulation} instances, allowing it to collect game pieces.
     */
    public void startIntake() {
        if (intakeRunning)
            return;

        driveTrainSimulation.addFixture(this);
        this.intakeRunning = true;
    }

    /**
     *
     *
     * <h2>Turns the Intake Off.</h2>
     *
     * <p>Retracts the intake into the chassis, removing it from the chassis's collision space.
     *
     * <p>Once turned off, the intake will no longer listen for or respond to contacts with {@link
     * GamePieceOnFieldSimulation} instances.
     */
    public void stopIntake() {
        if (!intakeRunning)
            return;

        driveTrainSimulation.removeFixture(this);
        this.intakeRunning = false;
    }

    /**
     *
     *
     * <h2>The {@link ContactListener} for the Intake Simulation.</h2>
     *
     * <p>This class can be added to the simulation world to detect and manage contacts between the
     * intake and {@link GamePieceOnFieldSimulation} instances of the type {@link
     * #targetedGamePieceType}.
     *
     * <p>If contact is detected and the intake is running, the {@link GamePieceOnFieldSimulation}
     * will be marked for removal from the field.
     */
    final class GamePieceContactListener implements ContactListener<Body> {
        @Override
        public void begin(ContactCollisionData<Body> collision, Contact contact) {
            if (!IntakeSimulation.this.intakeRunning)
                return;

            final CollisionBody<?> collisionBody1 = collision.getBody1();
            final CollisionBody<?> collisionBody2 = collision.getBody2();
            final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

            if (collisionBody1 instanceof GamePieceCollisionBody gamePiece
                    && acceptedGamePieceVariants.contains(gamePiece.gp.variant())
                    && fixture2 == IntakeSimulation.this) {
                intakeGamePiece(gamePiece);
            } else if (collisionBody2 instanceof GamePieceCollisionBody gamePiece
                    && acceptedGamePieceVariants.contains(gamePiece.gp.variant())
                    && fixture1 == IntakeSimulation.this) {
                intakeGamePiece(gamePiece);
            }
        }

        private void intakeGamePiece(GamePieceCollisionBody gamePiece) {
            IntakeSimulation sim = IntakeSimulation.this;
            switch (sim.intakeBehavior) {
                case ADD_HIGHEST_AVAILABLE -> sim.gamePieceStorage.addHighestAvailable(gamePiece.gp);
                case ADD_LOWEST_AVAILABLE -> sim.gamePieceStorage.addLowestAvailable(gamePiece.gp);
                case ADD_HIGHEST_DISPLACE -> sim.gamePieceStorage.addHighestDisplace(gamePiece.gp);
                case ADD_LOWEST_DISPLACE -> sim.gamePieceStorage.addLowestDisplace(gamePiece.gp);
                case ADD_RANDOM -> sim.gamePieceStorage.addRandom(gamePiece.gp);
            }
        }

        /* functions not used */
        @Override
        public void persist(ContactCollisionData<Body> collision, Contact oldContact, Contact newContact) {
        }

        @Override
        public void end(ContactCollisionData<Body> collision, Contact contact) {
        }

        @Override
        public void destroyed(ContactCollisionData<Body> collision, Contact contact) {
        }

        @Override
        public void collision(ContactCollisionData<Body> collision) {
        }

        @Override
        public void preSolve(ContactCollisionData<Body> collision, Contact contact) {
        }

        @Override
        public void postSolve(ContactCollisionData<Body> collision, SolvedContact contact) {
        }
    }

    GamePieceContactListener getGamePieceContactListener() {
        return new GamePieceContactListener();
    }
}