package org.ironmaple.simulation.gamepieces;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
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
 * {@link SimulatedArena#addGamePiece(GamePieceOnFieldSimulation)}.
 */
public class GamePieceOnFieldSimulation extends Body implements GamePieceInterface {
    public static final double COEFFICIENT_OF_FRICTION = 0.8, MINIMUM_BOUNCING_VELOCITY = 0.2;

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
     * <p>Affects the result of {@link SimulatedArena#getGamePiecesByType(String)}.
     */
    public final String type;

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Fixed Height.</h2>
     *
     * @param info info about the game piece type
     * @param initialPose the initial position of the game piece on the field
     */
    public GamePieceOnFieldSimulation(GamePieceInfo info, Pose2d initialPose) {
        this(info, () -> info.gamePieceHeight.in(Meters) / 2, initialPose, new Translation2d());
    }

    /**
     *
     *
     * <h2>Creates a Game Piece on the Field with Custom Height Supplier and Initial Velocity.</h2>
     *
     * @param info info about the game piece type
     * @param zPositionSupplier a supplier that provides the current Z-height of the game piece
     * @param initialPose the initial position of the game piece on the field
     * @param initialVelocityMPS the initial velocity of the game piece, in meters per second
     */
    public GamePieceOnFieldSimulation(
            GamePieceInfo info,
            DoubleSupplier zPositionSupplier,
            Pose2d initialPose,
            Translation2d initialVelocityMPS) {
        super();
        this.type = info.type;
        this.zPositionSupplier = zPositionSupplier;

        BodyFixture bodyFixture = super.addFixture(info.shape);

        bodyFixture.setFriction(COEFFICIENT_OF_FRICTION);
        bodyFixture.setRestitution(info.coefficientOfRestitution);
        bodyFixture.setRestitutionVelocity(MINIMUM_BOUNCING_VELOCITY);

        bodyFixture.setDensity(info.gamePieceMass.in(Kilogram) / info.shape.getArea());
        super.setMass(MassType.NORMAL);

        super.setLinearDamping(info.linearDamping);
        super.setAngularDamping(info.angularDamping);
        super.setBullet(true);

        super.setTransform(GeometryConvertor.toDyn4jTransform(initialPose));
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

    /**
     *
     *
     * <h2>Stores the info of a type of game piece</h2>
     *
     * @param type the type of the game piece, affecting categorization within the arena
     * @param shape the shape of the collision space for the game piece
     * @param gamePieceHeight the height (thickness) of the game piece, in meters
     * @param gamePieceMass the mass of the game piece, in kilograms
     */
    public record GamePieceInfo(
            String type,
            Convex shape,
            Distance gamePieceHeight,
            Mass gamePieceMass,
            double linearDamping,
            double angularDamping,
            double coefficientOfRestitution) {}

    public void onIntake(String intakeTargetGamePieceType) {}


    @Override
    public String getType() {
        return this.type;
    }

    @Override
    public Translation3d getVelocity3dMPS() {
        return new Translation3d(
                this.getLinearVelocity().getXComponent().x,
                this.getLinearVelocity().getYComponent().y,
                0);
    }

    @Override
    public boolean isGrounded() {
        return true;
    }
}
