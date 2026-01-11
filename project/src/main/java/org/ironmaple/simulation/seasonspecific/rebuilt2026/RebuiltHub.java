package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.*;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2026Rebuilt;

/**
 *
 *
 * <h2>Simulates a <strong>SPEAKER</strong>s on the field.</h2>
 *
 * <p>This class simulates the <strong>SPEAKER</strong>s on the field where <strong>NOTES</strong>s can be scored. It
 * may be amplified by using the {@link Arena2024Crescendo#activateAmp(boolean)} function on the host arena.
 */
public class RebuiltHub extends Goal {

    protected static final Translation3d redHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
    protected static final Translation3d blueHubPose = new Translation3d(11.938, 4.034536, 1.5748);
    StructPublisher<Pose3d> posePublisher;
    protected final Arena2026Rebuilt arena;

    /**
     *
     *
     * <h2>Creates an Speaker of the specified color.</h2>
     *
     * @param arena The host arena of this speaker.
     * @param isBlue Wether this is the blue speaker or the red one.
     */
    public RebuiltHub(Arena2026Rebuilt arena, boolean isBlue) {
        super(
                arena,
                Inches.of(47),
                Inches.of(47),
                Inches.of(10),
                "Fuel",
                isBlue ? blueHubPose : redHubPose,
                isBlue);

        arena = arena;
        StructPublisher<Pose3d> HubPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueHub" : "RedHub", Pose3d.struct)
                .publish();
        HubPosePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected boolean checkVel(GamePiece gamePiece) {
        return gamePiece.getVelocity3dMPS().getZ() > 0;
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        arena.addValueToMatchBreakdown(isBlue, "WastedFuel", arena.isAmped(isBlue) ? 1 : 0);
        arena.addToScore(isBlue, arena.isAmped(isBlue) ? 1 : 0);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        return;
    }
}
