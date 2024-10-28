package org.ironmaple.simulation;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.ironmaple.utils.ProjectileUtil.ProjectileDynamics;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * A base class used for all gamepieces in the simulation.
 * 
 * Used to keep continuity between the different states gamepieces can be in.
 */
public class GamePiece {
    public record GamePieceVariant(
        String type,
        double height,
        double mass,
        Convex shape,
        List<Pair<Translation3d, Translation3d>> targetVolumes,
        boolean placeOnFieldWhenTouchGround
    ) {
        @Override
        public final boolean equals(Object other) {
            return other instanceof GamePieceVariant
                && ((GamePieceVariant) other).type.equals(type);
        }
    }

    /**
     * A union type representing the different states a game piece can be in.
     */
    private sealed interface GamePieceState {
        default void onEnter(GamePiece gp, SimulatedArena arena) {};
        default void onExit(GamePiece gp, SimulatedArena arena) {};
        default void tick(GamePiece gp, SimulatedArena arena) {};
        Pose3d pose(GamePiece gp, SimulatedArena arena);
        GamePieceStateTag tag();

        public record Limbo() implements GamePieceState {
            @Override
            public Pose3d pose(GamePiece gp, SimulatedArena arena) {
                return new Pose3d(-1.0,-1.0,-1.0,new Rotation3d());
            }
            @Override
            public GamePieceStateTag tag() {
                return GamePieceStateTag.LIMBO;
            }
        }
        public record OnField(GamePieceCollisionBody body) implements GamePieceState {
            @Override
            public void onEnter(GamePiece gp, SimulatedArena arena) {
                arena.world().addBody(body);
            }
            @Override
            public void onExit(GamePiece gp, SimulatedArena arena) {
                arena.world().removeBody(body);
            }
            @Override
            public Pose3d pose(GamePiece gp, SimulatedArena arena) {
                var pose2d = GeometryConvertor.toWpilibPose2d(body.getTransform());
                var t = pose2d.getTranslation();
                return new Pose3d(
                    t.getX(),
                    t.getY(),
                    gp.variant.height / 2.0,
                    new Rotation3d(
                        0.0, 0.0, pose2d.getRotation().getRadians()
                    )
                );
            }
            @Override
            public GamePieceStateTag tag() {
                return GamePieceStateTag.ON_FIELD;
            }
        }
        public static final class InFlight implements GamePieceState {
            private final ProjectileDynamics dynamics;
            private final Timer timer = new Timer();
            private double lastTime = timer.get();
            private Pose3d pose;
            private Translation3d velocity;

            public InFlight(Pose3d pose, Translation3d velocity, ProjectileDynamics dynamics) {
                this.pose = pose;
                this.velocity = velocity;
                this.dynamics = dynamics;
            }
            @Override
            public void tick(GamePiece gp, SimulatedArena arena) {
                timer.start();
                double dt = Timer.getFPGATimestamp() - lastTime;
                lastTime = Timer.getFPGATimestamp();
                velocity = dynamics.calculate(dt, velocity);
                Twist3d twist = new Twist3d(
                    dt * velocity.getX(),
                    dt * velocity.getY(),
                    dt * velocity.getZ(),
                    0.0,
                    0.0,
                    0.0
                );
                pose = pose.exp(twist);
                if (timer.advanceIfElapsed(0.05)) {
                    gp.recordPose(pose);
                }
            }
            @Override
            public Pose3d pose(GamePiece gp, SimulatedArena arena) {
                return pose;
            }
            @Override
            public GamePieceStateTag tag() {
                return GamePieceStateTag.IN_FLIGHT;
            }
        }
        public record Held(Supplier<Pose3d> robotPoseSupplier, Supplier<Transform3d> offset) implements GamePieceState {
            @Override
            public Pose3d pose(GamePiece gp, SimulatedArena arena) {
                return robotPoseSupplier.get().transformBy(offset.get());
            }
            @Override
            public GamePieceStateTag tag() {
                return GamePieceStateTag.HELD;
            }
        }
    }

    /**
     * An enum representing the different states a game piece can be in.
     * 
     * <p>These states include:
     * <ul>
     * <li>{@link #LIMBO}: The game piece is not in play.
     * <li>{@link #ON_FIELD}: The game piece is on the field, available for intake.
     * <li>{@link #IN_FLIGHT}: The game piece is in the air, moving towards a target.
     * <li>{@link #HELD}: The game piece is being held by a robot.
     * </ul>
     */
    public enum GamePieceStateTag {
        /**
         * The game piece is not in play.
         */
        LIMBO,
        /**
         * The game piece is on the field, available for intake.
         */
        ON_FIELD,
        /**
         * The game piece is in the air, moving towards a target.
         */
        IN_FLIGHT,
        /**
         * The game piece is being held by a robot.
         */
        HELD
    }

    /**
     * An enum representing the different events a game piece can trigger.
     * 
     * <p>These events include:
     * <ul>
     * <li>{@link #TARGET_REACHED}: The game piece has reached a target,
     * this can only be triggered by a game piece in flight.
     * <li>{@link #LANDED}: The game piece has landed on the field,
     * this can only be triggered by a game piece in flight when it's Z coordinated hits the floor.
     * <li>{@link #INTAKEN}: The game piece has been intaken by a robot,
     * this can only be triggered by a game piece on the field.
     * <li>{@link #SHOT}: The game piece has been shot by a robot,
     * this can only be triggered by a game piece being held by a robot.
     * <li>{@link #EXPELLED}: The game piece has been expelled by a robot,
     * this can only be triggered by a game piece being held by a robot.
     * </ul>
     */
    public static enum GamePieceEvent {
        /**
         * The game piece has reached a target.
         * This can only be triggered by a game piece in flight
         * when it travels through a target volume.
         * 
         * <p>When this event is triggered, the game piece will transition to the {@link GamePieceStateTag#LIMBO} state.
         */
        TARGET_REACHED,
        /**
         * The game piece has landed on the field.
         * This can only be triggered by a game piece in flight
         * when it's Z coordinate hits the floor.
         * 
         * <p>When this event is triggered, the game piece will transition to the {@link GamePieceStateTag#ON_FIELD} state.
         */
        LANDED,
        /**
         * The game piece has been intaken by a robot.
         * This can only be triggered by a game piece on the field
         * when it's intaken by a robot.
         * 
         * <p>When this event is triggered, the game piece will transition to the {@link GamePieceStateTag#HELD} state.
         */
        INTAKEN,
        /**
         * The game piece has been shot by a robot.
         * This can only be triggered by a game piece being held by a robot.
         * 
         * <p>When this event is triggered, the game piece will transition to the {@link GamePieceStateTag#IN_FLIGHT} state.
         */
        SHOT,
        /**
         * The game piece has been expelled by a robot.
         * This can only be triggered by a game piece being held by a robot.
         * 
         * <p>When this event is triggered, the game piece will transition to the {@link GamePieceStateTag#ON_FIELD} state.
         */
        EXPELLED
    }

    @FunctionalInterface
    public interface GamePieceEventHandler {
        void handle(GamePieceVariant variant, Pose3d pose);
    }

    private final GamePieceVariant variant;
    private final SimulatedArena arena;
    private GamePieceState state = new GamePieceState.Limbo();
    private EnumMap<GamePieceEvent, ArrayList<GamePieceEventHandler>> eventHandlers = new EnumMap<>(GamePieceEvent.class) {
        {
            for (GamePieceEvent event : GamePieceEvent.values()) {
                put(event, new ArrayList<>());
            }
        }
    };
    private final CircularBuffer<Pose3d> poseHistory = new CircularBuffer<>(50);

    public GamePiece(GamePieceVariant variant, SimulatedArena arena) {
        this.variant = variant;
        this.arena = arena;
    }

    static class GamePieceCollisionBody extends Body {
        public final GamePieceVariant variant;
        public final BiConsumer<Supplier<Pose3d>, Supplier<Transform3d>> intakeCallback;
        private GamePieceCollisionBody(
            GamePieceVariant variant,
            BiConsumer<Supplier<Pose3d>, Supplier<Transform3d>> intakeCallback
        ) {
            super();
            this.variant = variant;
            this.intakeCallback = intakeCallback;
        }
    }

    private GamePieceCollisionBody createBody(Translation2d initialPosition, Translation2d initialVelocityMPS) {
        final double LINEAR_DAMPING = 3.5;
        final double ANGULAR_DAMPING = 5;
        final double COEFFICIENT_OF_FRICTION = 0.8;
        final double COEFFICIENT_OF_RESTITUTION = 0.3;
        final double MINIMUM_BOUNCING_VELOCITY = 0.2;

        var body = new GamePieceCollisionBody(variant, this::intake);

        BodyFixture bodyFixture = body.addFixture(variant.shape);

        bodyFixture.setFriction(COEFFICIENT_OF_FRICTION);
        bodyFixture.setRestitution(COEFFICIENT_OF_RESTITUTION);
        bodyFixture.setRestitutionVelocity(MINIMUM_BOUNCING_VELOCITY);

        bodyFixture.setDensity(variant.mass / variant.shape.getArea());
        body.setMass(MassType.NORMAL);

        body.translate(GeometryConvertor.toDyn4jVector2(initialPosition));

        body.setLinearDamping(LINEAR_DAMPING);
        body.setAngularDamping(ANGULAR_DAMPING);
        body.setBullet(true);

        body.setLinearVelocity(GeometryConvertor.toDyn4jVector2(initialVelocityMPS));

        return body;
    }

    private boolean insideVolume(Translation3d position, Pair<Translation3d, Translation3d> volume) {
        return position.getX() >= volume.getFirst().getX() && position.getX() <= volume.getSecond().getX()
            && position.getY() >= volume.getFirst().getY() && position.getY() <= volume.getSecond().getY()
            && position.getZ() >= volume.getFirst().getZ() && position.getZ() <= volume.getSecond().getZ();
    }

    private void transitionState(GamePieceState newState) {
        state.onExit(this, arena);
        state = newState;
        state.onEnter(this, arena);
    }

    private void recordPose(Pose3d pose) {
        poseHistory.addFirst(pose);
    }

    public Pose3d pose() {
        return state.pose(this, arena);
    }

    public GamePieceStateTag stateTag() {
        return state.tag();
    }

    public GamePieceVariant variant() {
        return variant;
    }

    public GamePiece addEventHandler(GamePieceEvent event, GamePieceEventHandler handler) {
        eventHandlers.get(event).add(handler);
        return this;
    }

    public void triggerEvent(GamePieceEvent event) {
        for (GamePieceEventHandler handler : eventHandlers.get(event)) {
            handler.handle(variant, this.pose());
        }
    }

    public GamePiece place(Translation2d initialPosition) {
        transitionState(new GamePieceState.OnField(createBody(initialPosition, new Translation2d())));
        return this;
    }

    public GamePiece slide(Translation2d initialPosition, Translation2d initialVelocityMPS) {
        transitionState(new GamePieceState.OnField(createBody(initialPosition, initialVelocityMPS)));
        return this;
    }

    public GamePiece launch(Pose3d initialPose, Translation3d initialVelocityMPS, ProjectileDynamics dynamics) {
        transitionState(new GamePieceState.InFlight(initialPose, initialVelocityMPS, dynamics));
        return this;
    }

    public GamePiece intake(Supplier<Pose3d> robotPoseSupplier, Supplier<Transform3d> offset) {
        transitionState(new GamePieceState.Held(robotPoseSupplier, offset));
        return this;
    }

    public void simulationSubTick() {
        state.tick(this, arena);
        if (state instanceof GamePieceState.InFlight) {
            var pose = state.pose(this, arena);
            if (pose.getTranslation().getZ() < 0.0) {
                triggerEvent(GamePieceEvent.LANDED);
                transitionState(new GamePieceState.OnField(createBody(
                    pose.getTranslation().toTranslation2d(),
                    new Translation2d()
                )));
            }
            if (variant.targetVolumes.stream().anyMatch(volume -> insideVolume(pose.getTranslation(), volume))) {
                triggerEvent(GamePieceEvent.TARGET_REACHED);
                transitionState(new GamePieceState.Limbo());
            }
        }
    }

    public void delete() {
        transitionState(new GamePieceState.Limbo());
    }
}
