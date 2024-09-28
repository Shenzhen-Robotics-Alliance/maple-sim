package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.collision.CollisionBody;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Segment;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListener;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import java.util.ArrayDeque;
import java.util.Queue;

import static org.ironmaple.utils.mathutils.GeometryConvertor.*;

public abstract class IntakeSimulation extends BodyFixture {
    private final int capacity;
    protected int gamePieceCount;
    private boolean intakeRunning;

    private final Queue<GamePieceOnFieldSimulation> gamePiecesToRemove;
    private final AbstractDriveTrainSimulation driveTrainSimulation;

    public enum IntakeSide {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }
    /**
     * Creates an intake simulation
     * the intake is considered a line segment on the robot
     * any game piece that touches the line will be grabbed
     *
     * @param width the valid width of the intake
     * @param side the side at which the intake is attached to
     * @param capacity the amount of game-pieces that can be hold in the intake
     */
    public IntakeSimulation(SimulatedArena arena, AbstractDriveTrainSimulation driveTrainSimulation, double width, IntakeSide side, int capacity) {
        this(
                arena,
                driveTrainSimulation,
                getIntakeRectangle(driveTrainSimulation, width, side),
                capacity
        );
    }

    private static Rectangle getIntakeRectangle(AbstractDriveTrainSimulation driveTrainSimulation, double with, IntakeSide side) {
        final Rectangle intakeRectangle = new Rectangle(with, 0.02);
        intakeRectangle.rotate(switch (side) {
            case LEFT, RIGHT -> 0;
            case FRONT, BACK -> Math.toRadians(90);
        });
        intakeRectangle.translate(switch (side) {
            case LEFT -> new Vector2(0, driveTrainSimulation.profile.width / 2);
            case RIGHT -> new Vector2(0, -driveTrainSimulation.profile.width / 2);
            case FRONT -> new Vector2(driveTrainSimulation.profile.length / 2, 0);
            case BACK -> new Vector2(-driveTrainSimulation.profile.length / 2, 0);
        });

        return intakeRectangle;
    };

    /**
     * Creates an intake simulation
     * the intake is considered a line segment on the robot
     * any game piece that touches the line will be grabbed
     *
     * @param startPointOnRobot     the start point of the segment, in relative to the robot
     * @param endPointOnRobot       the end point of the segment, in relative to the robot
     * @param capacity              the amount of game-pieces that can be hold in the intake
     */
    public IntakeSimulation(SimulatedArena arena, AbstractDriveTrainSimulation driveTrainSimulation, Translation2d startPointOnRobot, Translation2d endPointOnRobot, int capacity) {
        this(
                arena,
                driveTrainSimulation,
                new Segment(toDyn4jVector2(startPointOnRobot), toDyn4jVector2(endPointOnRobot)),
                capacity
        );
    }

    /**
     * Creates an intake simulation
     * the intake is fixed shape on the robot
     * any game piece that touches the line will be grabbed
     *
     * @param shape the shape of the intake
     * @param capacity              the amount of game-pieces that can be hold in the intake\
     */
    public IntakeSimulation(SimulatedArena arena, AbstractDriveTrainSimulation driveTrainSimulation, Convex shape, int capacity) {
        super(shape);
        if (capacity > 100)
            throw new IllegalArgumentException("capacity too large, max is 100");
        this.capacity = capacity;

        this.gamePiecesToRemove = new ArrayDeque<>(capacity);

        this.intakeRunning = false;
        this.driveTrainSimulation = driveTrainSimulation;
        arena.addIntakeSimulation(this);
    }

    protected void startIntake() {
        driveTrainSimulation.addFixture(this);
        this.intakeRunning = true;
    }

    protected void stopIntake() {
        driveTrainSimulation.removeFixture(this);
        this.intakeRunning = false;
    }

    public final class GamePieceContactListener implements ContactListener<Body> {
        @Override
        public void begin(ContactCollisionData collision, Contact contact) {
            if (!intakeRunning) return;
            if (gamePieceCount == capacity) return;

            final CollisionBody<?> collisionBody1 = collision.getBody1(), collisionBody2 = collision.getBody2();
            final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

            if (collisionBody1 instanceof GamePieceOnFieldSimulation && fixture2 == IntakeSimulation.this)
                flagGamePieceForRemoval((GamePieceOnFieldSimulation) collisionBody1);
            else if (collisionBody2 instanceof GamePieceOnFieldSimulation && fixture1 == IntakeSimulation.this)
                flagGamePieceForRemoval((GamePieceOnFieldSimulation) collisionBody2);
        }

        private void flagGamePieceForRemoval(GamePieceOnFieldSimulation gamePiece) {
            gamePiecesToRemove.add(gamePiece);
            gamePieceCount++;
        }

        /* functions not used */
        @Override public void persist(ContactCollisionData collision, Contact oldContact, Contact newContact) {}
        @Override public void end(ContactCollisionData collision, Contact contact) {}
        @Override public void destroyed(ContactCollisionData collision, Contact contact) {}
        @Override public void collision(ContactCollisionData collision) {}
        @Override public void preSolve(ContactCollisionData collision, Contact contact) {}
        @Override public void postSolve(ContactCollisionData collision, SolvedContact contact) {}
    }

    public GamePieceContactListener getGamePieceContactListener() {
        return new GamePieceContactListener();
    }

    public Queue<GamePieceOnFieldSimulation> getGamePiecesToRemove() {
        return gamePiecesToRemove;
    }

    public void clearGamePiecesToRemoveQueue() {
        this.gamePieceCount += gamePiecesToRemove.size();
        gamePiecesToRemove.clear();
    }
}
