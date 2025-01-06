package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class ReefscapeCoral extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo REEFSCAPE_CORAL_INFO =
            new GamePieceInfo("Coral", new Rectangle(0.3, 0.11), Meters.of(0.11), Kilograms.of(0.3), 1.8, 4, 0.3);

    public ReefscapeCoral(Pose2d initialPose) {
        super(REEFSCAPE_CORAL_INFO, initialPose);
    }
}
