package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h1>Simulates a Game Piece on the Field.</h1>
 *
 * <p>This class simulates a game piece on the field, which has a collision space and interacts with other objects.
 *
 * <p>Game pieces can be "grabbed" by an {@link IntakeSimulation}.
 *
 * <p>For the simulation to actually run, every instance must be added to a
 * {@link org.ironmaple.simulation.SimulatedArena} through
 * {@link SimulatedArena#addGamePiece(GamePieceOnField)}.
 */
public class GamePieceOnField extends Body {
    public static final double LINEAR_DAMPING = 3.5,
            ANGULAR_DAMPING = 5,
            COEFFICIENT_OF_FRICTION = 0.8,
            COEFFICIENT_OF_RESTITUTION = 0.3,
            MINIMUM_BOUNCING_VELOCITY = 0.2;

    /**
     *
     *
     * <h2>Supplier of the Current Z-Pose (Height) of the Game Piece.</h2>
     *
     * <p>Normally, the height is fixed at half the thickness of the game piece to simulate it being "on the ground."
     *
     * <p>If the game piece is flying at a low height, the height is calculated using the law of free-fall.
     */
    private final DoubleSupplier zPositionSupplier;
    /**
     *
     *
     * <h2>The Type of the Game Piece.</h2>
     *
     * <p>Affects the result of {@link SimulatedArena#getGamePiecePoses(String)}.
     */
    public final String type;

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Fixed Height.</h2>
     *
     * @param type the type of the game piece, affecting categorization within the arena
     * @param shape the shape of the collision space for the game piece
     * @param gamePieceHeight the height (thickness) of the game piece, in meters
     * @param mass the mass of the game piece, in kilograms
     * @param initialPosition the initial position of the game piece on the field
     */
    public GamePieceOnField(
            String type, Convex shape, double gamePieceHeight, double mass, Translation2d initialPosition) {
        this(type, shape, () -> gamePieceHeight / 2, mass, initialPosition, new Translation2d());
    }

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Custom Height Supplier and Initial Velocity.</h2>
     *
     * @param type the type of the game piece, affecting categorization within the arena
     * @param shape the shape of the collision space for the game piece
     * @param zPositionSupplier a supplier that provides the current Z-height of the game piece
     * @param mass the mass of the game piece, in kilograms
     * @param initialPosition the initial position of the game piece on the field
     * @param initialVelocityMPS the initial velocity of the game piece, in meters per second
     */
    public GamePieceOnField(
            String type,
            Convex shape,
            DoubleSupplier zPositionSupplier,
            double mass,
            Translation2d initialPosition,
            Translation2d initialVelocityMPS) {
        super();
        this.type = type;
        this.zPositionSupplier = zPositionSupplier;

        BodyFixture bodyFixture = super.addFixture(shape);

        bodyFixture.setFriction(COEFFICIENT_OF_FRICTION);
        bodyFixture.setRestitution(COEFFICIENT_OF_RESTITUTION);
        bodyFixture.setRestitutionVelocity(MINIMUM_BOUNCING_VELOCITY);

        bodyFixture.setDensity(mass / shape.getArea());
        super.setMass(MassType.NORMAL);

        super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));

        super.setLinearDamping(LINEAR_DAMPING);
        super.setAngularDamping(ANGULAR_DAMPING);
        super.setBullet(true);

        super.setLinearVelocity(GeometryConvertor.toDyn4jVector2(initialVelocityMPS));
    }

    /**
     *
     *
     * <h2>Sets the world velocity of this game piece.</h2>
     *
     * @param chassisSpeedsWorldFrame the speeds of the game piece
     */
    public void setVelocity(ChassisSpeeds chassisSpeedsWorldFrame) {
        super.setLinearVelocity(GeometryConvertor.toDyn4jLinearVelocity(chassisSpeedsWorldFrame));
        super.setAngularVelocity(chassisSpeedsWorldFrame.omegaRadiansPerSecond);
    }

    /**
     *
     *
     * <h2>Obtains the 2d position of the game piece</h2>
     *
     * @return the 2d position of the game piece
     */
    public Pose2d getPoseOnField() {
        return GeometryConvertor.toWpilibPose2d(super.getTransform());
    }

    /**
     *
     *
     * <h2>Obtains a 3d pose of the game piece.</h2>
     *
     * <p>The 3d position is calculated from both the {@link #getPoseOnField()} and {@link #zPositionSupplier}
     *
     * @return the 3d position of the game piece
     */
    public Pose3d getPose3d() {
        final Pose2d pose2d = getPoseOnField();
        return new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                zPositionSupplier.getAsDouble(),
                new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
    }
}
