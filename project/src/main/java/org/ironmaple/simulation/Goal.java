package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.List;
import java.util.function.Predicate;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.gamepieces.GamePiece;

/**
 *
 *
 * <h2>An Abstract Class to Handle Scoring Elements in Simulation.</h2>
 */
public abstract class Goal implements SimulatedArena.Simulatable {

    /**
     *
     *
     * <h2>Functional Interface for Position-Based Bound Checking</h2>
     *
     * <p>Validates whether a 3D position is within the goal's scoring zone.
     */
    @FunctionalInterface
    public interface PositionChecker {
        boolean isInBounds(Translation3d position);
    }

    /**
     *
     *
     * <h2>Functional Interface for Rotation-Based Validation</h2>
     *
     * <p>Validates whether a game piece's rotation is acceptable for scoring.
     */
    @FunctionalInterface
    public interface RotationChecker {
        boolean isValidRotation(GamePiece gamePiece);
    }

    /**
     *
     *
     * <h2>Reverses a Rotation 180 Degrees Around the Z-Axis</h2>
     *
     * <p>Used internally for checking game pieces that can score in either orientation.
     *
     * @param toFlip the rotation to flip
     * @return the flipped rotation
     */
    public static Rotation3d flipRotation(Rotation3d toFlip) {
        return new Rotation3d(0, -toFlip.getY(), toFlip.getZ() + Math.PI);
    }

    /**
     *
     *
     * <h2>Creates a Box-Shaped Position Checker</h2>
     *
     * <p>Validates that a position is within an axis-aligned 3D box.
     *
     * @param xyBox the 2D box in XY plane
     * @param minZMeters minimum Z coordinate in meters
     * @param maxZMeters maximum Z coordinate in meters
     * @return position checker for box-shaped bounds
     */
    public static PositionChecker box(Rectangle xyBox, double minZMeters, double maxZMeters) {
        return position -> xyBox.contains(new Vector2(position.getX(), position.getY()))
                && position.getZ() >= minZMeters
                && position.getZ() <= maxZMeters;
    }

    /**
     *
     *
     * <h2>Creates an Absolute Angle Rotation Checker</h2>
     *
     * <p>Validates that a game piece's rotation matches an expected angle within tolerance, checking both pitch and
     * yaw. Considers both normal and flipped (180-degree rotated) orientations.
     *
     * @param expectedAngle the expected 3D rotation
     * @param tolerance the allowed angular difference
     * @return rotation checker for absolute angle matching
     */
    public static RotationChecker absoluteAngle(Rotation3d expectedAngle, Angle tolerance) {
        return gamePiece -> {
            // Call our values just once.
            Rotation3d actualRotation = gamePiece.getPose3d().getRotation();
            Rotation3d normalDiff = actualRotation.minus(expectedAngle);
            Rotation3d flippedDiff = flipRotation(actualRotation).minus(expectedAngle);

            double normalAngleDegrees = new Rotation3d(
                            Degrees.of(0), normalDiff.getMeasureY(), normalDiff.getMeasureZ())
                    .getMeasureAngle()
                    .in(Units.Degrees);

            double flippedAngleDegrees = new Rotation3d(
                            Degrees.of(0), flippedDiff.getMeasureY(), flippedDiff.getMeasureZ())
                    .getMeasureAngle()
                    .in(Units.Degrees);

            return normalAngleDegrees < tolerance.in(Units.Degrees)
                    || flippedAngleDegrees < tolerance.in(Units.Degrees);
        };
    }

    /**
     *
     *
     * <h2>Creates a Pitch-Only Rotation Checker</h2>
     *
     * <p>Validates only the pitch (Y-axis rotation) of a game piece, ignoring yaw and roll. Useful for ring-shaped game
     * pieces where orientation around the vertical axis doesn't matter.
     *
     * @param expectedPitchRadians the expected pitch in radians
     * @param tolerance the allowed angular difference
     * @return rotation checker for pitch-only validation
     */
    public static RotationChecker pitchOnly(double expectedPitchRadians, Angle tolerance) {
        return gamePiece -> {
            double actualPitch = gamePiece.getPose3d().getRotation().getY();
            return Math.abs(actualPitch - expectedPitchRadians) < tolerance.in(Units.Radians);
        };
    }

    /**
     *
     *
     * <h2>Creates an Always-Valid Rotation Checker</h2>
     *
     * @return rotation checker that always returns true
     */
    public static RotationChecker anyRotation() {
        return gamePiece -> true;
    }

    protected Rectangle xyBox;
    protected final Distance height;
    protected final Distance elevation;

    protected final String gamePieceType;
    protected final Translation3d position;
    protected final SimulatedArena arena;
    protected final int max;
    public final boolean isBlue;
    protected int gamePieceCount = 0;

    protected RotationChecker rotationChecker;
    protected PositionChecker positionChecker;
    protected Predicate<GamePiece> velocityValidator;

    /**
     *
     *
     * <h2>Creates a Goal Object</h2>
     *
     * @param arena The host arena of this goal
     * @param xDimension The x dimension of the default box collider.
     * @param yDimension The y dimension of the default box collider.
     * @param height The height or z dimension of the default box collider.
     * @param gamePieceType the string game piece type to be handled by this goal.
     * @param position The position of this goal.
     * @param isBlue Whether this is a blue goal or a red one.
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

        this.gamePieceType = gamePieceType;
        this.position = position;
        this.arena = arena;
        this.max = max;
        this.isBlue = isBlue;
        this.height = height;
        this.elevation = Distance.ofBaseUnits(position.getZ(), Units.Meters);

        this.xyBox = new Rectangle(xDimension.in(Units.Meters), yDimension.in(Units.Meters));
        this.xyBox.translate(new Vector2(position.getX(), position.getY()));

        double minZMeters = position.getZ();
        double maxZMeters = position.getZ() + height.in(Units.Meters);

        this.rotationChecker = anyRotation();
        this.positionChecker = box(xyBox, minZMeters, maxZMeters);
        this.velocityValidator = (gamePiece) -> true;
    }

    /**
     *
     *
     * <h2>Creates a Goal Object with No Scoring Maximum</h2>
     *
     * @param arena The host arena of this goal.
     * @param xDimension The x dimension of the default box collider.
     * @param yDimension The y dimension of the default box collider.
     * @param height The height or z dimension of the default box collider.
     * @param gamePieceType the string game piece type to be handled by this goal.
     * @param position The position of this goal.
     * @param isBlue Whether this is a blue goal or a red one.
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
     * <h2>Sets the Angle to be Used When Checking Game Piece Rotation</h2>
     *
     * @param angle The angle that pieces should have when interacting with this goal
     * @param angleTolerance The tolerance to be used in checking said angle.
     */
    public void setNeededAngle(Rotation3d angle, Angle angleTolerance) {
        this.rotationChecker = absoluteAngle(angle, angleTolerance);
    }

    /**
     *
     *
     * <h2>Sets a Custom Rotation Checker for This Goal</h2>
     *
     * <p>Replaces the default rotation validation logic with a custom checker. This allows for complex rotation checks
     * beyond simple pitch/yaw tolerance, such as validating only specific axes or implementing custom logic.
     *
     * @param checker rotation checker that validates game piece rotation
     * @return this Goal instance for method chaining
     */
    public Goal withCustomRotationChecker(RotationChecker checker) {
        this.rotationChecker = checker;
        return this;
    }

    /**
     *
     *
     * <h2>Sets a Custom Rotation Validator for This Goal</h2>
     *
     * <p>Convenience method that accepts a Predicate for backward compatibility.
     *
     * @param validator predicate that accepts GamePiece and returns true if rotation is valid
     * @return this Goal instance for method chaining
     */
    protected boolean checkValidity(GamePiece gamePiece) {
        return rotationChecker.isValidRotation(gamePiece)
                && velocityValidator.test(gamePiece)
                && positionChecker.isInBounds(gamePiece.getPose3d().getTranslation());
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

    public Goal withCustomRotationValidator(Predicate<GamePiece> validator) {
        this.rotationChecker = validator::test;
        return this;
    }

    /**
     *
     *
     * <h2>Sets Rotation Tolerance for Pitch and Yaw</h2>
     *
     * <p>Configures the tolerance used by the default rotation validation logic. This only works if you have previously
     * set an expected angle using {@link #setNeededAngle(Rotation3d, Angle)}.
     *
     * <p>Note: This method cannot update the tolerance without knowing the expected angle. If you need to change the
     * tolerance, use {@link #setNeededAngle(Rotation3d, Angle)} again with the new tolerance.
     *
     * @param tolerance maximum allowed difference in degrees
     * @return this Goal instance for method chaining
     * @deprecated Use {@link #setNeededAngle(Rotation3d, Angle)} instead to set both angle and tolerance
     */
    @Deprecated
    public Goal withRotationTolerance(Angle tolerance) {
        return this;
    }

    /**
     *
     *
     * <h2>Sets a Custom Position Checker for This Goal</h2>
     *
     * <p>Replaces the default box collision with a custom shape check. This allows for cylindrical, spherical, or other
     * complex collision zones beyond axis-aligned boxes.
     *
     * @param checker position checker that validates 3D positions
     * @return this Goal instance for method chaining
     */
    public Goal withCustomPositionChecker(PositionChecker checker) {
        this.positionChecker = checker;
        return this;
    }

    /**
     *
     *
     * <h2>Sets a Custom Collision Detection Predicate</h2>
     *
     * <p>Convenience method that accepts a Predicate for backward compatibility.
     *
     * @param predicate function that accepts Translation3d and returns true if inside goal zone
     * @return this Goal instance for method chaining
     */
    public Goal withCustomCollisionPredicate(Predicate<Translation3d> predicate) {
        this.positionChecker = predicate::test;
        return this;
    }

    /**
     * Configures custom velocity requirements for scoring. This can be used to require pieces to be ascending,
     * descending, or moving within certain speed ranges.
     *
     * <p>The predicate receives the {@link GamePiece} and should return {@code true} if the velocity is acceptable for
     * scoring.
     *
     * @param validator predicate that accepts GamePiece and returns true if velocity is valid
     * @return this Goal instance for method chaining
     */
    public Goal withCustomVelocityValidator(Predicate<GamePiece> validator) {
        this.velocityValidator = validator;
        return this;
    }

    /**
     *
     *
     * <h2>Removes All Game Pieces from This Goal</h2>
     */
    public void clear() {
        this.gamePieceCount = 0;
    }

    /**
     *
     *
     * <h2>Returns Whether This Goal is Full</h2>
     *
     * @return Whether or not the goal is full.
     */
    public boolean isFull() {
        return this.gamePieceCount == this.max;
    }

    /**
     *
     *
     * <h2>Adds Points When a Piece Has Been Successfully Scored in This Goal</h2>
     *
     * <p>Since this function is the only trigger called when a piece is scored it may handle other small things outside
     * of adding points.
     */
    protected abstract void addPoints();

    /**
     *
     *
     * <h2>Displays Game Pieces to AdvantageScope if Applicable</h2>
     *
     * @param drawList a list of {@link Pose3d} objects used to visualize the positions of the game pieces on
     *     AdvantageScope
     */
    public abstract void draw(List<Pose3d> drawList);

    /**
     *
     *
     * <h2>Returns the Number of Game Pieces Currently Scored in This Goal</h2>
     *
     * @return This goals game piece count.
     */
    public int getGamePieceCount() {
        return gamePieceCount;
    }
}
