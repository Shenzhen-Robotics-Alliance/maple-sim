package org.ironmaple.simulation.seasonspecific.reefscape2025.reef;

import edu.wpi.first.math.geometry.*;
import java.util.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    private final Arena2025Reefscape arena;
    private final List<ReefscapeReefBranch> branches;
    private final List<ReefscapeReefSimulation.CoralHolder> coralHolders;

    public ReefscapeReefSimulation(Arena2025Reefscape arena) {
        this.arena = arena;
        this.coralHolders = new ArrayList<>(48);
        this.branches = new ArrayList<>(12);

        Translation2d origin =
                new Translation2d(FieldMirroringUtils.FIELD_WIDTH / 2, FieldMirroringUtils.FIELD_HEIGHT / 2);
        Translation2d[] branchesCenterPositionBlue = new Translation2d[] {
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
        Translation2d[] branchesCenterPositionRed = Arrays.stream(branchesCenterPositionBlue)
                .map(FieldMirroringUtils::flip)
                .toArray(Translation2d[]::new);
        Rotation2d[] branchesFacingOutwardsBlue = new Rotation2d[] {
            Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
            Rotation2d.fromDegrees(-120), Rotation2d.fromDegrees(-120), // C and D
            Rotation2d.fromDegrees(-60), Rotation2d.fromDegrees(-60), // E and F
            Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
            Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
            Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
        };
        Rotation2d[] branchesFacingOutwardsRed = Arrays.stream(branchesFacingOutwardsBlue)
                .map(FieldMirroringUtils::flip)
                .toArray(Rotation2d[]::new);

        ReefscapeReefBranch branch;
        for (int i = 0; i < 12; i++) {
            // blue
            branch = new ReefscapeReefBranch(branchesCenterPositionBlue[i], branchesFacingOutwardsBlue[i]);
            branches.add(branch);
            coralHolders.addAll(branch.coralHolders());

            // red
            branch = new ReefscapeReefBranch(branchesCenterPositionRed[i], branchesFacingOutwardsRed[i]);
            branches.add(branch);
            coralHolders.addAll(branch.coralHolders());
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        Set<GamePieceProjectile> gamePiecesLaunched = arena.gamePieceLaunched();
        Set<GamePieceProjectile> toRemoves = new HashSet<>();
        for (ReefscapeReefSimulation.CoralHolder coralHolder : coralHolders)
            for (GamePieceProjectile gamePieceLaunched : gamePiecesLaunched)
                checkForCoralPlacement(coralHolder, gamePieceLaunched, toRemoves);

        for (GamePieceProjectile toRemove : toRemoves) gamePiecesLaunched.remove(toRemove);
    }

    private void checkForCoralPlacement(
            ReefscapeReefSimulation.CoralHolder coralHolder,
            GamePieceProjectile gamePieceLaunched,
            Set<GamePieceProjectile> toRemove) {
        if (gamePieceLaunched instanceof ReefscapeCoralOnFly coralOnFly)
            if (!toRemove.contains(gamePieceLaunched) && coralHolder.checkCoralPlacement(coralOnFly))
                toRemove.add(gamePieceLaunched);
    }

    public void addCoralsOnReefForDisplay(List<Pose3d> coralPosesToDisplay) {
        for (ReefscapeReefSimulation.CoralHolder coralHolder : coralHolders)
            coralHolder.addContainedCoralsForDisplay(coralPosesToDisplay);
    }

    public void clearReef() {
        for (ReefscapeReefBranch branch : branches) branch.clearBranch();
    }

    protected interface CoralHolder {
        boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly);

        void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay);
    }
}
