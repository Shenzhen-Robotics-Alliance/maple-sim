package org.ironmaple.simulation.seasonspecific.reefscape2025.reef;

import edu.wpi.first.math.geometry.*;
import java.util.List;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class ReefscapeReefTrough implements ReefscapeReefSimulation.CoralHolder {
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
