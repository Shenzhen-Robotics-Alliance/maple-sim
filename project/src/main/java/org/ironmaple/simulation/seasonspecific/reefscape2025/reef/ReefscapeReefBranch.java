package org.ironmaple.simulation.seasonspecific.reefscape2025.reef;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Collection;
import java.util.List;

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
        L1.coralCount = 0;
        L2.hasCoral = L3.hasCoral = L4.hasCoral = false;
    }

    public Collection<ReefscapeReefSimulation.CoralHolder> coralHolders() {
        return List.of(L1, L2, L3, L4);
    }
}
