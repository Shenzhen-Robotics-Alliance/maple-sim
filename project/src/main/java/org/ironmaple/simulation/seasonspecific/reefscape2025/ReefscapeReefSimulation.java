package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

/**
 *
 *
 * <h2>Simulates the two <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates the two <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored. It
 * includes all 12 {@link ReefscapeReefBranchesTower} instances on the field (both blue and red).
 */
public class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    private final Arena2025Reefscape arena;
    private final List<ReefscapeReefBranchesTower> branchTowers;
    private final List<CoralHolder> coralHolders;

    public ReefscapeReefSimulation(Arena2025Reefscape arena) {
        this.arena = arena;
        this.coralHolders = new ArrayList<>(96);
        this.branchTowers = new ArrayList<>(24);

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

        ReefscapeReefBranchesTower branch;
        for (int i = 0; i < 12; i++) {
            // blue
            branch = new ReefscapeReefBranchesTower(branchesCenterPositionBlue[i], branchesFacingOutwardsBlue[i]);
            branchTowers.add(branch);
            coralHolders.addAll(branch.coralHolders());

            // red
            branch = new ReefscapeReefBranchesTower(branchesCenterPositionRed[i], branchesFacingOutwardsRed[i]);
            branchTowers.add(branch);
            coralHolders.addAll(branch.coralHolders());
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        Set<GamePieceProjectile> gamePiecesLaunched = arena.gamePieceLaunched();
        Set<GamePieceProjectile> toRemoves = new HashSet<>();
        for (CoralHolder coralHolder : coralHolders)
            for (GamePieceProjectile gamePieceLaunched : gamePiecesLaunched)
                checkForCoralPlacement(coralHolder, gamePieceLaunched, toRemoves);

        for (GamePieceProjectile toRemove : toRemoves) gamePiecesLaunched.remove(toRemove);
    }

    private void checkForCoralPlacement(
            CoralHolder coralHolder, GamePieceProjectile gamePieceLaunched, Set<GamePieceProjectile> toRemove) {
        if (gamePieceLaunched instanceof ReefscapeCoralOnFly coralOnFly)
            if (!toRemove.contains(gamePieceLaunched) && coralHolder.checkCoralPlacement(coralOnFly))
                toRemove.add(gamePieceLaunched);
                
    }

    /**
     * Displays all the CORALs scored on the REEF.
     *
     * @param coralPosesToDisplay a list of {@link Pose3d} objects used to visualize the positions of the CORALs on
     *     AdvantageScope
     */
    public void addCoralsOnReefForDisplay(List<Pose3d> coralPosesToDisplay) {
        for (CoralHolder coralHolder : coralHolders) coralHolder.addContainedCoralsForDisplay(coralPosesToDisplay);
    }

    /** Clears all the CORALs scored on the REEF. */
    public void clearReef() {
        for (ReefscapeReefBranchesTower tower : branchTowers) tower.clearBranches();
    }

    /**
     * Obtains the amount of <strong>CORAL</strong> held on the <strong>BRANCHES</strong>.
     *
     * <p>This method returns a 2D array of size 12 x 4, where each entry represents the number of
     * <strong>CORAL</strong>s held on a particular branch.
     *
     * <p>The <strong>BRANCHES</strong> are tracked in FMS as A, B, C, D, E, F, G, H, I, J, K, L (as per the game
     * manual), and are mapped to indices 0, 1, 2, ... in the array.
     *
     * <p>The [i][j] entry in the array represents the number of <strong>CORAL</strong>(s) held on the L<code>j-1</code>
     * branch in the <code>i</code>th section.
     *
     * <p>For example, <code>getBranches()[2][3]</code> returns the number of CORALs held on L4 of Branch C.
     *
     * <p>Note that L2, L3, and L4 can only hold one <strong>CORAL</strong>, while L1 can hold up to two
     * <strong>CORAL</strong>s.
     *
     * @param side the alliance side (Red or Blue) to check for CORAL counts
     * @return a 2D array where each entry represents the number of <strong>CORAL</strong> held on each branch
     */
    public int[][] getBranches(DriverStation.Alliance side) {
        int[][] coralsCountOnBranches = new int[12][4];
        for (int i = 0; i < 12; i++) {
            ReefscapeReefBranchesTower tower = branchTowers.get(side == DriverStation.Alliance.Red ? i * 2 + 1 : i * 2);
            coralsCountOnBranches[i][0] = tower.L1.coralCount;
            coralsCountOnBranches[i][1] = tower.L2.hasCoral ? 1 : 0;
            coralsCountOnBranches[i][2] = tower.L3.hasCoral ? 1 : 0;
            coralsCountOnBranches[i][3] = tower.L4.hasCoral ? 1 : 0;
        }

        return coralsCountOnBranches;
    }

    /**
     * Returns an optional instance.
     *
     * @return (optionally) an instance of this class, empty if
     */
    public static Optional<ReefscapeReefSimulation> getInstance() {
        if (SimulatedArena.getInstance() instanceof Arena2025Reefscape arena2025Reefscape)
            return Optional.of(arena2025Reefscape.reefSimulation);
        return Optional.empty();
    }
}

/**
 *
 *
 * <h1>Simulates a BRANCHES tower in the REEF.</h1>
 *
 * <p>A BRANCHES TOWER is a purple tower containing:
 *
 * <ul>
 *   <li>The L1 TROUGH.
 *   <li>The L2, L3, and L4 BRANCHES.
 * </ul>
 */
class ReefscapeReefBranchesTower {
    public final ReefscapeReefTrough L1;
    public final ReefscapeReefBranch L2, L3, L4;

    public ReefscapeReefBranchesTower(Translation2d stickCenterPositionOnField, Rotation2d facingOutwards) {
        // L1 trough, 15cm away from center
        this.L1 = new ReefscapeReefTrough(
                stickCenterPositionOnField.plus(new Translation2d(0.15, facingOutwards)), facingOutwards);

        // L2 stick, 20 cm away from center, 78cm above ground, 35 deg pitch
        this.L2 = new ReefscapeReefBranch(
                stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
                facingOutwards,
                0.77,
                Math.toRadians(-35));

        // L3 stick, 20 cm away from center, 118cm above ground, 35 deg pitch
        this.L3 = new ReefscapeReefBranch(
                stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
                facingOutwards,
                1.17,
                Math.toRadians(-35));

        // L4 stick, 30 cm away from center, 178cm above ground, vertical
        this.L4 = new ReefscapeReefBranch(
                stickCenterPositionOnField.plus(new Translation2d(0.26, facingOutwards)),
                facingOutwards,
                1.78,
                Math.toRadians(-90));
    }

    /** Clears all the CORALs on the BRANCHES */
    public void clearBranches() {
        L1.coralCount = 0;
        L2.hasCoral = L3.hasCoral = L4.hasCoral = false;
    }

    /** Obtains a collection of the {@link CoralHolder}s that are in this BRANCHES tower. */
    public Collection<CoralHolder> coralHolders() {
        return List.of(L1, L2, L3, L4);
    }
}

/**
 *
 *
 * <h2>Represents a BRANCH or a TROUGH.</h2>
 *
 * <p>Represents a structure on which <strong>CORAL</strong>s can be scored.
 */
sealed interface CoralHolder permits ReefscapeReefBranch, ReefscapeReefTrough {
    /**
     * Checks if a CORAL has been successfully scored on this holder.
     *
     * @param coralOnFly the {@link ReefscapeCoralOnFly} to check for placement
     * @return true if the CORAL is successfully scored, false otherwise
     */
    boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly);

    /**
     * Displays the positions of the CORALs that are on this holder.
     *
     * @param coralPosesToDisplay a list of poses used to visualize CORALs on AdvantageScope
     */
    void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay);
}

/**
 *
 *
 * <h1>Simulates a BRANCH (L2, L3, or L4) on a {@link ReefscapeReefBranchesTower}.</h1>
 *
 * <p>If a <strong>CORAL</strong> ejected into the air is in contact with this <strong>BRANCH</strong> at the correct
 * angle, as detected by {@link #checkCoralPlacement(ReefscapeCoralOnFly)}, it will get attached.
 */
final class ReefscapeReefBranch implements CoralHolder {
    private final Pose3d idealCoralPlacementPose;
    public boolean hasCoral;

    ReefscapeReefBranch(
            Translation2d idealPlacementPosition,
            Rotation2d facingOutwards,
            double heightMeters,
            double branchInwardsDirectionPitchRad) {
        this.idealCoralPlacementPose = new Pose3d(
                idealPlacementPosition.getX(),
                idealPlacementPosition.getY(),
                heightMeters,
                new Rotation3d(
                        0,
                        -branchInwardsDirectionPitchRad,
                        facingOutwards.plus(Rotation2d.k180deg).getRadians()));
        this.hasCoral = false;
    }

    private static final double TRANSLATIONAL_TOLERANCE_METERS = 0.06;
    private static final double ROTATIONAL_TOLERANCE_RADIANS = Math.toRadians(50);

    /**
     *
     *
     * <h2>Checks if a CORAL has been successfully scored on this BRANCH.</h2>
     *
     * <p>A CORAL is successfully placed on a BRANCH if the following conditions are met:
     *
     * <ul>
     *   <li>This <strong>BRANCH</strong> is not already holding any other CORAL.
     *   <li>The <strong>CORAL</strong> is detected to be in physical contact with the BRANCH in space, within a
     *       specified tolerance.
     *   <li>The <strong>CORAL</strong>'s opening is aligned with the direction of the BRANCH's tip, ensuring proper
     *       placement.
     *   <li>The <strong>CORAL</strong> is moving in a direction that is towards the inside of the BRANCH, not away from
     *       it.
     * </ul>
     *
     * <p>In more technical terms, the CORAL's placement is validated by checking if the distance between the CORAL's
     * current position and the ideal placement pose is within a predefined tolerance. Additionally, the CORAL's
     * velocity and direction of movement are checked to ensure it is moving towards the correct placement orientation.
     *
     * <p>If these conditions are met, the CORAL is considered successfully placed on the BRANCH.
     *
     * @param coralOnFly the {@link ReefscapeCoralOnFly} object to check for placement on the BRANCH
     * @return <code>true</code> if the CORAL is successfully placed onto this BRANCH, <code>false</code> otherwise
     */
    @Override
    public boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly) {
        Twist3d positionDifference = coralOnFly.getPose3d().log(idealCoralPlacementPose);
        Translation3d velocityMPS = coralOnFly.getVelocity3dMPS();

        boolean goingDown = velocityMPS.getZ() <= 0;
        boolean positionCorrection =
                Math.hypot(positionDifference.dy, positionDifference.dz) < TRANSLATIONAL_TOLERANCE_METERS
                        && Math.abs(positionDifference.dx) < 0.15;

        Translation3d idealCoralFacingVector = new Translation3d(1, idealCoralPlacementPose.getRotation());
        Translation3d actualCoralFacingVector =
                new Translation3d(1, coralOnFly.getPose3d().getRotation());
        double rotationToleranceVectorNorm = new Translation2d(1, Rotation2d.fromRadians(ROTATIONAL_TOLERANCE_RADIANS))
                .minus(new Translation2d(1, 0))
                .getNorm();
        boolean directionCorrect =
                idealCoralFacingVector.minus(actualCoralFacingVector).getNorm() < rotationToleranceVectorNorm
                        || idealCoralFacingVector
                                        .times(-1)
                                        .minus(actualCoralFacingVector)
                                        .getNorm()
                                < rotationToleranceVectorNorm;
        boolean targetHit = positionCorrection && goingDown && (!this.hasCoral);
        if (!targetHit) return false;

        return this.hasCoral = true;
    }

    @Override
    public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
        if (hasCoral) coralPosesToDisplay.add(idealCoralPlacementPose);
    }
}

/**
 *
 *
 * <h1>Simulates a TROUGH (L1) on a {@link ReefscapeReefBranchesTower}.</h1>
 *
 * <p>If a <strong>CORAL</strong> ejected into the air is in contact with this <strong>TROUGH</strong> at the correct
 * angle, as detected by {@link #checkCoralPlacement(ReefscapeCoralOnFly)}, it will get attached.
 */
final class ReefscapeReefTrough implements CoralHolder {
    private final Pose3d firstPlacementPose, secondPlacementPose;
    private final Translation3d idealPlacementPosition;
    public int coralCount;

    protected ReefscapeReefTrough(Translation2d centerPosition, Rotation2d outwardsFacing) {
        Rotation3d coralRotation =
                new Rotation3d(0, 0, outwardsFacing.plus(Rotation2d.kCCW_90deg).getRadians());
        Translation2d firstPosition = centerPosition.plus(new Translation2d(0.08, outwardsFacing));
        Translation2d secondPosition = centerPosition.plus(new Translation2d(-0.04, outwardsFacing));
        this.firstPlacementPose = new Pose3d(firstPosition.getX(), firstPosition.getY(), 0.48, coralRotation);
        this.secondPlacementPose = new Pose3d(secondPosition.getX(), secondPosition.getY(), 0.52, coralRotation);
        this.idealPlacementPosition = new Translation3d(centerPosition.getX(), centerPosition.getY(), 0.47);
        this.coralCount = 0;
    }

    /**
     * Checks if a CORAL has been successfully placed on this holder.
     *
     * <p>A CORAL is successfully placed if it is within a specified distance from the ideal placement position and
     * within a reasonable height range.
     *
     * <p>The placement is considered successful if the following conditions are met:
     *
     * <ul>
     *   <li>The CORAL is close enough to the ideal placement position (within 0.25 units horizontally and 0.1 units
     *       vertically).
     *   <li>The CORAL can only be placed once, as the holder allows a maximum of two CORALs.
     * </ul>
     *
     * @param coralOnFly the {@link ReefscapeCoralOnFly} object to check for placement
     * @return <code>true</code> if the CORAL is successfully placed, <code>false</code> otherwise
     */
    @Override
    public boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly) {
        if (coralCount >= 2) return false;

        Translation3d difference = coralOnFly.getPose3d().getTranslation().minus(idealPlacementPosition);
        boolean closeEnough = difference.toTranslation2d().getNorm() < 0.25 && Math.abs(difference.getZ()) < 0.1;

        if (closeEnough) coralCount++;
        return closeEnough;
    }

    @Override
    public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
        if (coralCount > 0) coralPosesToDisplay.add(firstPlacementPose);
        if (coralCount > 1) coralPosesToDisplay.add(secondPlacementPose);
    }
}
