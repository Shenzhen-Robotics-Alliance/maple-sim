package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.gamepieces.GamePiece;

/**
 *
 *
 * <h2>A abstract class to handle scoring elements in simulation.</h2>
 */
public abstract class Goal implements SimulatedArena.Simulatable {

    protected Rectangle xyBox;
    protected final Distance height;
    protected final Distance elevation;

    protected final String gamePieceType;

    protected final Translation3d position;
    protected final SimulatedArena arena;
    protected final int max;
    public final boolean isBlue;
    protected int gamePieceCount = 0;
    protected Rotation3d pieceAngle = null;
    protected Angle pieceAngleTolerance = Angle.ofBaseUnits(15, Degrees);

    /**
     *
     *
     * <h2>Reverses a rotation so that the returned rotation points directly "behind" the original rotation.</h2>
     *
     * @param toFlip The rotation to be flipped.
     * @return The flipped rotation.
     */
    public static Rotation3d flipRotation(Rotation3d toFlip) {
        return new Rotation3d(0, -toFlip.getY(), toFlip.getZ() + Math.PI);
    }

    /**
     *
     *
     * <h2>Creates a goal object </h2>
     *
     * @param arena The host arena of this goal
     * @param xDimension The x dimension of the default box collider.
     * @param yDimension The y dimension of the default box collider.
     * @param height The height or z dimension of the default box collider.
     * @param gamePieceType the string game piece type to be handled by this goal.
     * @param position The position of this goal.
     * @param isBlue Wether this is a blue goal or a red one.
     * @param max How many pieces can be scored in this goal.
     */
    public Goal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            String gamePieceType,
            Translation3d position,
            boolean isBlue,
            int max) {

        xyBox = new Rectangle(xDimension.in(Units.Meters), yDimension.in(Units.Meters));
        this.height = height;
        this.gamePieceType = gamePieceType;
        this.position = position;
        this.arena = arena;
        this.max = max;
        this.elevation = position.getMeasureZ();
        this.isBlue = isBlue;

        xyBox.translate(new Vector2(position.getX(), position.getY()));
    }

    /**
     *
     *
     * <h2>Creates a goal object with no scoring max.</h2>
     *
     * @param arena The host arena of this goal.
     * @param xDimension The x dimension of the default box collider.
     * @param yDimension The y dimension of the default box collider.
     * @param height The height or z dimension of the default box collider.
     * @param gamePieceType the string game piece type to be handled by this goal.
     * @param position The position of this goal.
     * @param isBlue Wether this is a blue goal or a red one.
     */
    public Goal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            String gamePieceType,
            Translation3d position,
            boolean isBlue) {
        this(arena, xDimension, yDimension, height, gamePieceType, position, isBlue, 99999);
    }

    /**
     *
     *
     * <h2>Handles the update triggers for the goal object.</h2>
     */
    @Override
    public void simulationSubTick(int subTickNum) {
        if (gamePieceCount >= max) return; // Early exit if already at max
        /// Use list filtering for more efficient bulk checking.
        // Get all pieces of our type as a list.
        arena.getGamePiecesByType(gamePieceType).stream()
                .filter(gamePiece -> !checkGrounded(gamePiece))
                .filter(this::checkValidity)
                .limit(max - gamePieceCount) // Only score what we can
                .forEach(
                        gamePiece -> { // If a piece passes, score it.
                            gamePieceCount++;
                            this.addPoints();
                            arena.removePiece(gamePiece);
                        });
    }

    /**
     *
     *
     * <h2>Sets the angle to be used when checking game piece rotation.
     *
     * @param angle The angle that pieces should have when interacting with this goal
     * @param angleTolerance The tolerance to be used in checking said angle.
     */
    public void setNeededAngle(Rotation3d angle, Angle angleTolerance) {
        pieceAngle = angle;
        pieceAngleTolerance = angleTolerance;
    }

    /**
     *
     *
     * <h2>Sets the angle to be used when checking game piece rotation.
     *
     * @param angle The angle that pieces should have when interacting with this goal
     */
    public void setNeededAngle(Rotation3d angle) {
        setNeededAngle(angle, pieceAngleTolerance);
    }

    /**
     * A high level call to check wether or not the provided piece is within this goals hit box and meets all
     * requirements to be scored. For more information on how this function calculates see the functions:
     *
     * <p>{@link org.dyn4j.geometry.AbstractShape#contains(Vector2 point)}
     *
     * <p>{@link Goal#checkRotation(GamePiece)}
     *
     * <p>{@link Goal#checkVel(GamePiece)}
     *
     * <p>Be aware that due to the nature of the goal class the above functions may or may not have the same
     * implementation for children of the goal class.
     *
     * @param gamePiece The game piece to be checked.
     * @return Wether or not the game piece is within this goal.
     */
    protected boolean checkValidity(GamePiece gamePiece) {
        return checkRotation(gamePiece) & checkVel(gamePiece) & checkCollision(gamePiece);
    }

    /**
     *
     *
     * <h2>Checks whether the submitted game piece is grounded </h2>
     *
     * @param gamePiece The game piece to have its groundedness checked.
     * @return Whether the piece is grounded.
     */
    protected boolean checkGrounded(GamePiece gamePiece) {
        return gamePiece.isGrounded();
    }

    /**
     *
     *
     * <h2>Checks wether or not the submitted game piece has a rotation able to be scored in this goal </h2>
     *
     * By default the rotation needed and tolerance may be set using the {@link Goal#setNeededAngle(Rotation3d, Angle)}
     * function. However rotation checks may be handled differently by some children making this not apply. Additionally
     * be aware that this function only supports pitch and yaw, not role. If support for roll is needed a custom
     * implementation will have to be created.
     *
     * @param gamePiece The game piece to have its rotation checked.
     * @return Wether or not the pieces rotation is consistent with rotation that are able to be scored in this goal.
     */
    protected boolean checkRotation(GamePiece gamePiece) {
        if (pieceAngle == null) {
            return true;
        }

        Rotation3d normalDiff = gamePiece.getPose3d().getRotation().minus(pieceAngle);
        Rotation3d flippedDiff =
                flipRotation(gamePiece.getPose3d().getRotation()).minus(pieceAngle);

        return new Rotation3d(Degrees.of(0), normalDiff.getMeasureZ(), normalDiff.getMeasureZ())
                                .getMeasureAngle()
                                .in(Degrees)
                        < pieceAngleTolerance.in(Degrees)
                || new Rotation3d(Degrees.of(0), flippedDiff.getMeasureZ(), flippedDiff.getMeasureZ())
                                .getMeasureAngle()
                                .in(Degrees)
                        < pieceAngleTolerance.in(Degrees);
    }

    /**
     *
     *
     * <h2>Returns wether or not the submitted game piece is within the goal. </h2>
     *
     * @param gamePiece The game piece to be checked.
     * @return Wether or not the game piece is within the goal.
     */
    protected boolean checkCollision(GamePiece gamePiece) {
        // Call our values just once.
        var pose = gamePiece.getPose3d();
        double minZ = elevation.in(Units.Meters);
        double maxZ = minZ + height.in(Units.Meters);

        return xyBox.contains(new Vector2(pose.getX(), pose.getY())) && pose.getZ() >= minZ && pose.getZ() <= maxZ;
    }

    /**
     *
     *
     * <h2>Function to check wether the velocity of potential game pieces is acceptable.</h2>
     *
     * The default implementation of this function always returns true so any velocity checks will need to be
     * implemented by children classes.
     *
     * @param gamePiece The game piece to have its velocity checked.
     * @return Wether or not the pieces velocity is consistent with velocities that are able to be scored in this goal.
     */
    protected boolean checkVel(GamePiece gamePiece) {
        return true;
    }

    /**
     *
     *
     * <h2>Removes all game pieces from this goal </h2>
     */
    public void clear() {
        this.gamePieceCount = 0;
    }

    /**
     *
     *
     * <h2>Returns wether or not this goal is full.</h2>
     *
     * @return wether or not the goal is full.
     */
    public boolean isFull() {
        return this.gamePieceCount == this.max;
    }

    /**
     *
     *
     * <h2>Adds points when a piece has been successfully scored in this goal</h2>
     *
     * Since this function is the only trigger called when a piece is scored it may handle other small things outside of
     * adding points.
     */
    protected abstract void addPoints();

    /**
     *
     *
     * <h2>Displays game pieces to advantage scope if applicable.</h2>
     *
     * @param drawList a list of {@link Pose3d} objects used to visualize the positions of the game pieces on
     *     AdvantageScope
     */
    public abstract void draw(List<Pose3d> drawList);

    /**
     *
     *
     * <h2>Returns the number of game pieces currently scored in this goal.</h2>
     *
     * @return This goals game piece count.
     */
    public int getGamePieceCount() {
        return gamePieceCount;
    }
}
