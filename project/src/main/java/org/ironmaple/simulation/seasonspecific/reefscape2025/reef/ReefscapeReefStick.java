package org.ironmaple.simulation.seasonspecific.reefscape2025.reef;

import edu.wpi.first.math.geometry.*;
import java.util.List;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

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

    private static final double TRANSLATIONAL_TOLERANCE_METERS = 0.055;
    private static final double ROTATIONAL_TOLERANCE_RADIANS = Math.toRadians(15);
    private static final double VELOCITY_DIRECTION_PITCH_TOLERANCE_RADIANS = Math.toRadians(30);

    @Override
    public boolean checkCoralPlacement(ReefscapeCoralOnFly coralOnFly) {
        Transform3d difference = idealCoralPlacementPose.minus(coralOnFly.getPose3d());
        Translation3d velocityMPS = coralOnFly.getVelocity3dMPS();
        double velocityDirectionPitchRad =
                Math.atan2(velocityMPS.getZ(), velocityMPS.toTranslation2d().getNorm());

        boolean velocityIgnorable = velocityMPS.getNorm() < 0.5;
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
