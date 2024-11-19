package org.ironmaple.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collector;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.ProjectileUtil.ProjectileDynamics;
import org.ironmaple.utils.geometry.Velocity2d;
import org.ironmaple.utils.geometry.Velocity3d;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;

/**
 * A base class used for all gamepieces in the simulation.
 * 
 * Used to keep continuity between the different states gamepieces can be in.
 */
public class GamePiece {
    /**
     * Represents a reactangular prism volume in 3d space.
     * 
     * <p> For implementation simplicity the volume supports yaw scewing but not roll or pitch.
     */
    public record GamePieceTarget(Rectangle2d area, Pair<Double, Double> heightRange) {
        public GamePieceTarget(Translation3d first, Translation3d second) {
            this(new Rectangle2d(first.toTranslation2d(), second.toTranslation2d()),
                    new Pair<>(first.getZ(), second.getZ()));
        }

        public boolean isInside(Translation3d position) {
            return area.contains(position.toTranslation2d())
                    && position.getZ() >= heightRange.getFirst()
                    && position.getZ() <= heightRange.getSecond();
        }
    }

    /**
     * An object describing the properties of a {@link GamePiece} variant.
     */
    public record GamePieceVariant(
            String type,
            double height,
            double mass,
            Convex shape,
            List<GamePieceTarget> targets,
            boolean placeOnFieldWhenTouchGround,
            double landingDampening) {
        @Override
        public final boolean equals(Object other) {
            return other instanceof GamePieceVariant
                    && ((GamePieceVariant) other).type.equals(type);
        }

        public boolean isOfVariant(GamePiece gp) {
            return gp.isOfVariant(this);
        }
    }

    /**
     * A union type representing the different states a game piece can be in.
     */
    protected sealed interface GamePieceStateData {
        /**
         * Called when a game piece enters this state.
         * @param gp the game piece entering this state
         * @param arena the arena the game piece is in
         */
        default void onEnter(GamePiece gp, SimulationArena arena) {
        };

        /**
         * Called when a game piece exits this state.
         * @param gp the game piece exiting this state
         * @param arena the arena the game piece is in
         */
        default void onExit(GamePiece gp, SimulationArena arena) {
        };

        /**
         * Called every simulation tick when the game piece is in this state.
         * @param gp the game piece in this state
         * @param arena the arena the game piece is in
         */
        default void tick(GamePiece gp, SimulationArena arena) {
        };

        /**
         * Returns the pose of the game piece in this state.
         * @param gp the game piece in this state
         * @param arena the arena the game piece is in
         * @return the pose of the game piece in this state
         */
        Pose3d pose(GamePiece gp, SimulationArena arena);

        /**
         * Returns the tag of this state.
         * @return the tag of this state
         */
        GamePieceState tag();

        /**
         * A state representing a game piece that is not in play
         * but still *exists*.
         */
        public record Limbo() implements GamePieceStateData {
            @Override
            public Pose3d pose(GamePiece gp, SimulationArena arena) {
                return new Pose3d(-1.0, -1.0, -1.0, new Rotation3d());
            }

            @Override
            public GamePieceState tag() {
                return GamePieceState.LIMBO;
            }
        }

        /**
         * A state representing a game piece that is on the field and available for intake.
         */
        public record OnField(GamePieceCollisionBody body) implements GamePieceStateData {
            @Override
            public void onEnter(GamePiece gp, SimulationArena arena) {
                arena.world().addBody(body);
            }

            @Override
            public void onExit(GamePiece gp, SimulationArena arena) {
                arena.world().removeBody(body);
            }

            @Override
            public Pose3d pose(GamePiece gp, SimulationArena arena) {
                var pose2d = GeometryConvertor.toWpilibPose2d(body.getTransform());
                var t = pose2d.getTranslation();
                return new Pose3d(
                        t.getX(),
                        t.getY(),
                        gp.variant.height / 2.0,
                        new Rotation3d(
                                0.0, 0.0, pose2d.getRotation().getRadians()));
            }

            @Override
            public GamePieceState tag() {
                return GamePieceState.ON_FIELD;
            }
        }

        /**
         * A state representing a game piece that is in the air, moving towards a target.
         */
        public static final class InFlight implements GamePieceStateData {
            private final ProjectileDynamics dynamics;
            private Pose3d pose;
            private Velocity3d velocity;

            public InFlight(Pose3d pose, Velocity3d velocity, ProjectileDynamics dynamics) {
                this.pose = pose;
                this.velocity = velocity;
                this.dynamics = dynamics;
            }

            @Override
            public void tick(GamePiece gp, SimulationArena arena) {
                double dt = arena.timing.dt;
                velocity = dynamics.calculate(dt, velocity);
                Twist3d twist = new Twist3d(
                        dt * velocity.getVX(),
                        dt * velocity.getVY(),
                        dt * velocity.getVZ(),
                        0.0,
                        0.0,
                        0.0);
                pose = pose.exp(twist);
            }

            @Override
            public Pose3d pose(GamePiece gp, SimulationArena arena) {
                return pose;
            }

            @Override
            public GamePieceState tag() {
                return GamePieceState.IN_FLIGHT;
            }
        }

        /**
         * A state representing a game piece that is being held by a robot.
         */
        public record Held(Supplier<Pose3d> robotPoseSupplier, Supplier<Transform3d> offset)
                implements GamePieceStateData {
            @Override
            public Pose3d pose(GamePiece gp, SimulationArena arena) {
                return robotPoseSupplier.get().transformBy(offset.get());
            }

            @Override
            public GamePieceState tag() {
                return GamePieceState.HELD;
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
    public enum GamePieceState {
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
        HELD;

        public boolean isInState(GamePiece gp) {
            return gp.isInState(this);
        }
    }

    /**
     * A class representing the collision body of a game piece.
     * This is used to provide extra info in the dyn4j functions when doing collision handling.
     */
    protected static class GamePieceCollisionBody extends Body {
        public final GamePiece gp;

        private GamePieceCollisionBody(GamePiece gp) {
            super();
            this.gp = gp;
        }

        protected static GamePieceCollisionBody createBody(GamePiece gp, Translation2d initialPosition,
                Velocity2d initialVelocity) {
            final double LINEAR_DAMPING = 3.5;
            final double ANGULAR_DAMPING = 5;
            final double COEFFICIENT_OF_FRICTION = 0.8;
            final double COEFFICIENT_OF_RESTITUTION = 0.3;
            final double MINIMUM_BOUNCING_VELOCITY = 0.2;

            var body = new GamePieceCollisionBody(gp);

            BodyFixture bodyFixture = body.addFixture(gp.variant.shape);

            bodyFixture.setFriction(COEFFICIENT_OF_FRICTION);
            bodyFixture.setRestitution(COEFFICIENT_OF_RESTITUTION);
            bodyFixture.setRestitutionVelocity(MINIMUM_BOUNCING_VELOCITY);

            bodyFixture.setDensity(gp.variant.mass / gp.variant.shape.getArea());
            body.setMass(MassType.NORMAL);

            body.translate(GeometryConvertor.toDyn4jVector2(initialPosition));

            body.setLinearDamping(LINEAR_DAMPING);
            body.setAngularDamping(ANGULAR_DAMPING);
            body.setBullet(true);

            body.setLinearVelocity(GeometryConvertor.toDyn4jVector2(initialVelocity));

            return body;
        }
    }

    protected final GamePieceVariant variant;
    protected final SimulationArena arena;
    protected GamePieceStateData state = new GamePieceStateData.Limbo();
    protected boolean userControlled = false;

    public GamePiece(GamePieceVariant variant, SimulationArena arena) {
        this.variant = variant;
        this.arena = arena;
    }

    protected void transitionState(GamePieceStateData newState) {
        state.onExit(this, arena);
        state = newState;
        state.onEnter(this, arena);
        RuntimeLog.debug("GamePiece: Transitioned to state " + state.tag());
    }

    /**
     * @return the pose of the {@link GamePiece} in the simulation
     */
    public Pose3d pose() {
        return state.pose(this, arena);
    }

    /**
     * @return a tag representing the state of the {@link GamePiece}
     */
    public GamePieceState state() {
        return state.tag();
    }

    /**
     * @return the {@link GamePieceVariant} of the {@link GamePiece}
     */
    public GamePieceVariant variant() {
        return variant;
    }

    public boolean isInState(GamePieceState... tags) {
        return List.of(tags).contains(state.tag());
    }

    public boolean isOfVariant(GamePieceVariant... variants) {
        return List.of(variants).contains(variant);
    }

    /**
     * Allows the user to control the {@link GamePiece}.
     * 
     * @return this {@link GamePiece} object
     */
    GamePiece userControlled() {
        userControlled = true;
        return this;
    }

    GamePiece withLib(Consumer<GamePiece> consumer) {
        boolean prevControl = userControlled;
        userControlled = true;
        consumer.accept(this);
        userControlled = prevControl;
        return this;
    }

    /**
     * Releases control of the {@link GamePiece} back to the library only.
     * 
     * <p> The user should never have to call this method but it is not dangerous to do so
     * therefore it is public.
     */
    public void releaseControl() {
        userControlled = false;
    }

    /**
     * Returns whether the {@link GamePiece} is user controlled.
     * 
     * @return whether the {@link GamePiece} is user controlled
     */
    public boolean isUserControlled() {
        return userControlled;
    }

    public boolean isLibraryControlled() {
        return !userControlled;
    }

    /**
     * Intakes the {@link GamePiece} if the user has control of the {@link GamePiece}.
     * 
     * @param robotPoseSupplier a supplier for the pose of the robot
     * @param offset a supplier for the offset of the game piece from the robot
     */
    public void intake(Supplier<Pose3d> robotPoseSupplier, Supplier<Transform3d> offset) {
        if (userControlled) {
            transitionState(new GamePieceStateData.Held(robotPoseSupplier, offset));
            releaseControl();
        } else {
            RuntimeLog.warn("Tried to intake a game piece without control");
        }
    }

    /**
     * Places the {@link GamePiece} on the field at the given position
     * if the user has control of the {@link GamePiece}.
     * 
     * @param pose the position to place the {@link GamePiece} at
     */
    public void place(Translation2d pose) {
        if (userControlled) {
            transitionState(
                    new GamePieceStateData.OnField(GamePieceCollisionBody.createBody(this, pose, new Velocity2d())));
            releaseControl();
        } else {
            RuntimeLog.warn("Tried to place a game piece without control");
        }
    }

    /**
     * Slides the {@link GamePiece} on the field at the given position and velocity
     * if the user has control of the {@link GamePiece}.
     * 
     * @param initialPosition the position to place the {@link GamePiece} at
     * @param initialVelocity the velocity to slide the {@link GamePiece} at
     */
    public void slide(Translation2d initialPosition, Velocity2d initialVelocity) {
        if (userControlled) {
            transitionState(new GamePieceStateData.OnField(
                    GamePieceCollisionBody.createBody(this, initialPosition, initialVelocity)));
            releaseControl();
        } else {
            RuntimeLog.warn("Tried to slide a game piece without control");
        }
    }

    /**
     * Launches the {@link GamePiece} from the given pose and velocity
     * if the user has control of the {@link GamePiece}.
     * 
     * @param initialPose the pose to launch the {@link GamePiece} from
     * @param initialVelocity the velocity to launch the {@link GamePiece} at
     * @param dynamics the dynamics of the projectile
     */
    public void launch(Pose3d initialPose, Velocity3d initialVelocity, ProjectileDynamics dynamics) {
        if (userControlled) {
            transitionState(new GamePieceStateData.InFlight(initialPose, initialVelocity, dynamics));
            releaseControl();
        } else {
            RuntimeLog.warn("Tried to launch a game piece without control");
        }
    }

    /**
     * Deletes the {@link GamePiece} if the user has control of the {@link GamePiece}.
     */
    public void delete() {
        if (userControlled) {
            transitionState(new GamePieceStateData.Limbo());
            releaseControl();
        } else {
            RuntimeLog.warn("Tried to delete a game piece without control");
        }
    }

    void simulationSubTick() {
        state.tick(this, arena);
        if (state instanceof GamePieceStateData.InFlight state) {
            var pose = state.pose(this, arena);
            if (pose.getTranslation().getZ() < 0.0) {
                transitionState(new GamePieceStateData.OnField(GamePieceCollisionBody.createBody(
                        this,
                        pose.getTranslation().toTranslation2d(),
                        state.velocity.toVelocity2d().times(variant.landingDampening))));
                RuntimeLog.debug("GamePiece: Landed");
            }
            if (variant.targets.stream().anyMatch(target -> target.isInside(pose.getTranslation()))) {
                transitionState(new GamePieceStateData.Limbo());
                userControlled();
                RuntimeLog.debug("GamePiece: Target reached");
            }
        }
    }

    public static Collector<GamePiece, ArrayList<Pose3d>, Pose3d[]> poseStreamCollector() {
        return new Collector<GamePiece, ArrayList<Pose3d>, Pose3d[]>() {
            @Override
            public Supplier<ArrayList<Pose3d>> supplier() {
                return () -> new ArrayList<>();
            }

            @Override
            public BiConsumer<ArrayList<Pose3d>, GamePiece> accumulator() {
                return (poses, gamePiece) -> poses.add(gamePiece.pose());
            }

            @Override
            public Set<Characteristics> characteristics() {
                return Set.of();
            }

            @Override
            public BinaryOperator<ArrayList<Pose3d>> combiner() {
                return (poses1, poses2) -> {
                    poses1.addAll(poses2);
                    return poses1;
                };
            }

            @Override
            public Function<ArrayList<Pose3d>, Pose3d[]> finisher() {
                return poses -> poses.toArray(new Pose3d[0]);
            }
        };
    }
}
