package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.Goal;
import org.ironmaple.utils.FieldMirroringUtils;

/**
 *
 *
 * <h2>Simulates a Reefscape branch on the field.</h2>
 *
 * <p>This class simulates a Reefscape branch where corals can be scored. This class should not be used directly and
 * instead should be used via a {@link ReefscapeReefSimulation} which will handle an entire reef.
 */
public class ReefscapeReefBranch extends Goal {

    public final int level;
    public final int column;

    private static Angle angleTolerance = Degrees.of(10);
    private static final List<ReefscapeReefBranch> allInstances = new ArrayList<>();

    /**
     *
     *
     * <h2>Sets the Angle Tolerance for All Reef Branches</h2>
     *
     * <p>This configures the tolerance used for rotation validation across all reef branch instances. Changes take
     * effect immediately for all instances, including those already created.
     *
     * @param tolerance The angle tolerance to use for all branches
     */
    public static void setAngleTolerance(Angle tolerance) {
        angleTolerance = tolerance;
        // Update all existing instances
        for (ReefscapeReefBranch branch : allInstances) 
            branch.updateRotationChecker();
    }

    public static final Translation2d origin =
            new Translation2d(FieldMirroringUtils.FIELD_WIDTH / 2, FieldMirroringUtils.FIELD_HEIGHT / 2);
    public static final Translation2d[] branchesCenterPositionBlue = new Translation2d[] {
        new Translation2d(-4.810, 0.164).plus(origin), // A
        new Translation2d(-4.810, -0.164).plus(origin), // B
        new Translation2d(-4.690, -0.373).plus(origin), // C
        new Translation2d(-4.406, -0.538).plus(origin), // D
        new Translation2d(-4.164, -0.537).plus(origin), // E
        new Translation2d(-3.879, -0.374).plus(origin), // F
        new Translation2d(-3.759, -0.164).plus(origin), // G
        new Translation2d(-3.759, 0.164).plus(origin), // H
        new Translation2d(-3.880, 0.373).plus(origin), // I
        new Translation2d(-4.164, 0.538).plus(origin), // J
        new Translation2d(-4.405, 0.538).plus(origin), // K
        new Translation2d(-4.690, 0.374).plus(origin) // L
    };

    public static final Translation3d[] heights = new Translation3d[] {
        new Translation3d(0, 0, 0.45),
        new Translation3d(0, 0, 0.79),
        new Translation3d(0, 0, 1.19),
        new Translation3d(0, 0, 1.78)
    };

    public static final Rotation3d flip90 = new Rotation3d(0, 0, Math.PI / 2);

    public static final Translation2d[] branchesCenterPositionRed = Arrays.stream(branchesCenterPositionBlue)
            .map(FieldMirroringUtils::flip)
            .toArray(Translation2d[]::new);

    public static final Rotation2d[] branchesFacingOutwardsBlue = new Rotation2d[] {
        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
        Rotation2d.fromDegrees(240), Rotation2d.fromDegrees(240), // C and D
        Rotation2d.fromDegrees(300), Rotation2d.fromDegrees(300), // E and F
        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
        Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
        Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
    };

    /**
     *
     *
     * <h2>Returns the required pose of a reef branch at the designated position.</h2>
     *
     * @param isBlue Wether the position is on the blue reef or the red reef.
     * @param level The level of the reef (0 indexed). Range of 0-3.
     * @param col The pole or Colum of the reef (0 indexed). Range of 0-11.
     * @return The pose of a reef branch with the specified stats.
     */
    public static Translation3d getPoseOfBranchAt(boolean isBlue, int level, int col) {
        Translation3d branchCenterPosition = isBlue
                ? new Translation3d(branchesCenterPositionBlue[col])
                : new Translation3d(branchesCenterPositionRed[col]);

        return branchCenterPosition
                .plus(heights[level])
                .plus(new Translation3d(0.255, 0, 0).rotateBy(new Rotation3d(branchesFacingOutwardsBlue[col])));
    }

    public static final Rotation2d[] branchesFacingOutwardsRed = Arrays.stream(branchesFacingOutwardsBlue)
            .map(FieldMirroringUtils::flip)
            .toArray(Rotation2d[]::new);

    private static final int MIN_LEVEL = 0;
    private static final int MAX_LEVEL = 3;

    /**
     *
     *
     * <h2>Creates a singular reef branch at the specified location </h2>
     *
     * @param arena The host arena of this reef.
     * @param isBlue Wether the position is on the blue reef or the red reef.
     * @param level The level of the reef (0 indexed). Range of 0-3.
     * @param column The pole or Colum of the reef (0 indexed). Range of 0-11.
     */
    public ReefscapeReefBranch(Arena2025Reefscape arena, boolean isBlue, int level, int column) {
        super(
                arena,
                level == 0 ? Centimeters.of(30) : Centimeters.of(10),
                level == 0 ? Centimeters.of(100) : Centimeters.of(10),
                Centimeters.of(30),
                "Coral",
                getPoseOfBranchAt(isBlue, level, column),
                isBlue,
                level == 0 ? 2 : 1);

        if (level < MIN_LEVEL || level > MAX_LEVEL) {
            throw new IllegalArgumentException(
                    "Invalid reef level: " + level + " (must be between " + MIN_LEVEL + " and " + MAX_LEVEL + ")");
        }

        this.level = level;
        this.column = column;

        // Set initial rotation checker
        updateRotationChecker();

        // Register this instance for future tolerance updates
        allInstances.add(this);
    }

    /**
     *
     *
     * <h2>Updates the Rotation Checker Based on Current Angle Tolerance</h2>
     *
     * <p>This method is called during construction and whenever the static angle tolerance is changed.
     */
    private void updateRotationChecker() {
        switch (level) {
            case 1, 2 -> {
                Rotation3d targetAngle = new Rotation3d(
                        Degrees.zero(),
                        Degrees.of(-35),
                        (isBlue ? branchesFacingOutwardsBlue : branchesFacingOutwardsRed)[column].getMeasure());
                setNeededAngle(targetAngle, angleTolerance);
            }
            case 3 -> setNeededAngle(new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero()), angleTolerance);
        } // case 0 uses default anyRotation() from parent
    }

    /**
     *
     *
     * <h2>Gives the pose of the reef branch.</h2>
     *
     * @return This position of this branch as a pose3d.
     */
    public Pose3d getPose() {
        Rotation3d facing = new Rotation3d(
                0,
                level == 1 || level == 2 ? -35 * Math.PI / 180 : 0,
                (isBlue ? branchesFacingOutwardsBlue : branchesFacingOutwardsRed)[column].getRadians());

        if (level == 0) {
            facing = facing.plus(flip90);
        }

        return new Pose3d(position, facing);
    }

    private static final int[] AUTO_POINTS_BY_LEVEL = {3, 4, 6, 7};
    private static final int[] TELEOP_POINTS_BY_LEVEL = {2, 3, 4, 5};

    @Override
    protected void addPoints() {
        System.out.println("Coral scored on level: " + (level + 1) + " on the " + (isBlue ? "Blue " : "Red") + "reef");
        arena.addValueToMatchBreakdown(isBlue, "Auto/CoralScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);
        arena.addValueToMatchBreakdown(isBlue, "CoralScoredOnLevel " + String.valueOf(level + 1), 1);

        int points = DriverStation.isAutonomous() ? AUTO_POINTS_BY_LEVEL[level] : TELEOP_POINTS_BY_LEVEL[level];
        arena.addToScore(isBlue, points);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        if (this.gamePieceCount > 0) {
            drawList.add(getPose());
        }
        if (this.gamePieceCount > 1) {
            drawList.add(getPose().transformBy(new Transform3d(0, isBlue ? 0.12 : -0.12, 0, new Rotation3d())));
        }
    }
}
