package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeReefSimulation implements SimulatedArena.Simulatable {
    private final Arena2025Reefscape arena;
    private final List<ReefscapeReefBranch> branches;
    private final List<CoralHolder> coralHolders;

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

    public void addCoralsOnReefForDisplay(List<Pose3d> coralPosesToDisplay) {
        for (CoralHolder coralHolder : coralHolders) coralHolder.addContainedCoralsForDisplay(coralPosesToDisplay);
    }

    public void clearReef() {
        for (ReefscapeReefBranch branch : branches) branch.clearBranch();
    }

    protected interface CoralHolder {
        boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly);

        void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay);
    }
}

class ReefscapeReefStick implements ReefscapeReefSimulation.CoralHolder {
    private final Pose3d idealCoralPlacementPose;
    private final double idealVelocityDirectionPitchToScoreRad;
    public boolean hasCoral;

    protected ReefscapeReefStick(
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
        this.idealVelocityDirectionPitchToScoreRad = branchInwardsDirectionPitchRad;
        this.hasCoral = false;
    }

    private static final double TRANSLATIONAL_TOLERANCE_METERS = 0.05;
    private static final double ROTATIONAL_TOLERANCE_RADIANS = Math.toRadians(6);
    private static final double VELOCITY_DIRECTION_PITCH_TOLERANCE_RADIANS = Math.toRadians(25);

    @Override
    public boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly) {
        Transform3d difference = idealCoralPlacementPose.minus(coralOnFly.getPose3d());
        Translation3d velocityMPS = coralOnFly.getVelocity3dMPS();
        double velocityDirectionPitchRad =
                Math.atan2(velocityMPS.getZ(), velocityMPS.toTranslation2d().getNorm());

        boolean velocityIgnorable = velocityMPS.getNorm() < 0.6;
        boolean velocityDirectionCorrect = Math.abs(velocityDirectionPitchRad - idealVelocityDirectionPitchToScoreRad)
                < VELOCITY_DIRECTION_PITCH_TOLERANCE_RADIANS;
        boolean targetHit = isWithinTolerance(difference, velocityIgnorable, velocityDirectionCorrect);
        if (!targetHit) return false;

        this.hasCoral = true;
        return true;
    }

    private boolean isWithinTolerance(
            Transform3d difference, boolean velocityIgnorable, boolean velocityDirectionCorrect) {
        boolean poseWithinTolerance = difference.getTranslation().getNorm() < TRANSLATIONAL_TOLERANCE_METERS
                && Math.abs(difference.getRotation().getX()) < ROTATIONAL_TOLERANCE_RADIANS
                && Math.abs(difference.getRotation().getY()) < ROTATIONAL_TOLERANCE_RADIANS
                && Math.abs(difference.getRotation().getZ()) < ROTATIONAL_TOLERANCE_RADIANS;
        boolean targetHit = (velocityIgnorable || velocityDirectionCorrect) && poseWithinTolerance && (!this.hasCoral);
        return targetHit;
    }

    @Override
    public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
        if (hasCoral) coralPosesToDisplay.add(idealCoralPlacementPose);
    }
}

class ReefscapeReefTrough implements ReefscapeReefSimulation.CoralHolder {
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

class ReefscapeReefBranch {
    public final ReefscapeReefTrough L1;
    public final ReefscapeReefStick L2, L3, L4;

    public ReefscapeReefBranch(Translation2d stickCenterPositionOnField, Rotation2d facingOutwards) {
        // L1 trough, 15cm away from center
        this.L1 = new ReefscapeReefTrough(
                stickCenterPositionOnField.plus(new Translation2d(0.15, facingOutwards)), facingOutwards);

        // L2 stick, 20 cm away from center, 78cm above ground, 35 deg pitch
        this.L2 = new ReefscapeReefStick(
                stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
                facingOutwards,
                0.77,
                Math.toRadians(-35));

        // L3 stick, 20 cm away from center, 118cm above ground, 35 deg pitch
        this.L3 = new ReefscapeReefStick(
                stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
                facingOutwards,
                1.17,
                Math.toRadians(-35));

        // L4 stick, 30 cm away from center, 178cm above ground, vertical
        this.L4 = new ReefscapeReefStick(
                stickCenterPositionOnField.plus(new Translation2d(0.26, facingOutwards)),
                facingOutwards,
                1.78,
                Math.toRadians(-90));
    }

    public void clearBranch() {
        //        L1.coralCount = 0;
        //        L2.hasCoral = L3.hasCoral = L4.hasCoral = false;
        L1.coralCount = 2;
        L2.hasCoral = L3.hasCoral = L4.hasCoral = true;
    }

    public Collection<ReefscapeReefSimulation.CoralHolder> coralHolders() {
        return List.of(L1, L2, L3, L4);
    }
}

class ReefscapeCoralOnFly extends GamePieceProjectile {
    public ReefscapeCoralOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(
                ReefscapeCoral.REEFSCAPE_CORAL_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
