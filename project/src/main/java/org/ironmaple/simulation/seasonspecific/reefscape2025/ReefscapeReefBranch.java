package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.goal;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeReefBranch extends goal {

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
        new Translation3d(0, 0, 0.15),
        new Translation3d(0, 0, 0.77),
        new Translation3d(0, 0, 1.17),
        new Translation3d(0, 0, 1.78)
    };

    public static final Translation2d[] branchesCenterPositionRed = Arrays.stream(branchesCenterPositionBlue)
            .map(FieldMirroringUtils::flip)
            .toArray(Translation2d[]::new);
    public static final Rotation2d[] branchesFacingOutwardsBlue = new Rotation2d[] {
        Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
        Rotation2d.fromDegrees(-120), Rotation2d.fromDegrees(-120), // C and D
        Rotation2d.fromDegrees(-60), Rotation2d.fromDegrees(-60), // E and F
        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
        Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
        Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
    };

    public static Translation3d getPose(boolean isBlue, int level, int col) {
        if (isBlue) {
            return new Translation3d(branchesCenterPositionBlue[col]).plus(heights[level]);
        } else {
            return new Translation3d(branchesCenterPositionRed[col]).plus(heights[level]);
        }
    }

    public static final Rotation2d[] branchesFacingOutwardsRed = Arrays.stream(branchesFacingOutwardsBlue)
            .map(FieldMirroringUtils::flip)
            .toArray(Rotation2d[]::new);

    // ReefscapeReefBranchesTower branch;
    // for (int i = 0; i < 12; i++) {
    //     // blue
    //     branch = new ReefscapeReefBranchesTower(branchesCenterPositionBlue[i], branchesFacingOutwardsBlue[i]);
    //     branchTowers.add(branch);
    //     coralHolders.addAll(branch.coralHolders());

    //     // red
    //     branch = new ReefscapeReefBranchesTower(branchesCenterPositionRed[i], branchesFacingOutwardsRed[i]);
    //     branchTowers.add(branch);
    //     coralHolders.addAll(branch.coralHolders());

    public final int level;

    public ReefscapeReefBranch(Arena2025Reefscape arena, boolean isBlue, int level, int col) {
        super(
                arena,
                Centimeters.of(10),
                Centimeters.of(10),
                Centimeters.of(10),
                "Coral",
                getPose(isBlue, level, col),
                isBlue);

        if (level == 1 || level == 2) {
            if (isBlue) {
                setNeededVelAngle(
                        new Rotation3d(0, -35 * Math.PI / 180, branchesFacingOutwardsBlue[col].getRadians()),
                        Degrees.of(10));
            } else {
                setNeededVelAngle(
                        new Rotation3d(0, -35 * Math.PI / 180, branchesFacingOutwardsRed[col].getRadians()),
                        Degrees.of(10));
            }
        } else if (level == 3) {
            setNeededVelAngle(
                    new Rotation3d(0, -Math.PI / 2, branchesFacingOutwardsRed[col].getRadians()), Degrees.of(10));
        }
        this.level = level;
        ;
    }

    @Override
    protected void addPoints() {
        if (isBlue) {
            if (DriverStation.isAutonomous()) {
                switch (level) {
                    case 4:
                        arena.addToBlueScore(7);
                    case 3:
                        arena.addToBlueScore(6);
                    case 2:
                        arena.addToBlueScore(4);
                    case 1:
                        arena.addToBlueScore(3);
                }
            } else {
                switch (level) {
                    case 4:
                        arena.addToBlueScore(5);
                    case 3:
                        arena.addToBlueScore(4);
                    case 2:
                        arena.addToBlueScore(3);
                    case 1:
                        arena.addToBlueScore(2);
                }
            }
        } else {
            if (DriverStation.isAutonomous()) {
                switch (level) {
                    case 4:
                        arena.addToRedScore(7);
                    case 3:
                        arena.addToRedScore(6);
                    case 2:
                        arena.addToRedScore(4);
                    case 1:
                        arena.addToRedScore(3);
                }
            } else {
                switch (level) {
                    case 4:
                        arena.addToRedScore(5);
                    case 3:
                        arena.addToRedScore(4);
                    case 2:
                        arena.addToRedScore(3);
                    case 1:
                        arena.addToRedScore(2);
                }
            }
        }
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        if (this.gamePieceCount == 1) {
            drawList.add(new Pose3d(position, peiceAngle));
        }
    }
}
