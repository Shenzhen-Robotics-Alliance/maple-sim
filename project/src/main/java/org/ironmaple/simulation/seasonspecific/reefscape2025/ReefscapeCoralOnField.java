package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 *
 *
 * <h1>Represents a CORAL in the 2025 Reefscape game.</h1>
 *
 * <p>The Coral (<a href="https://www.andymark.com/products/frc-2024?sku=am-5601">am-5601</a>) is an 11 ⅞-inch (~30 cm)
 * long piece of 4-inch diameter Schedule 40 Cellular (Foam) Core PVC pipe, featured as a game piece in the 2025
 * Reefscape game.
 *
 * <p>The Coral has a 4-inch (~102 mm) inside diameter and a 4½-inch (~11 cm) outside diameter.
 */
public class ReefscapeCoralOnField extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo REEFSCAPE_CORAL_INFO =
            new GamePieceInfo("Coral", new Rectangle(0.3, 0.11), Meters.of(0.11), Kilograms.of(0.3), 2.8, 4, 0.3);

    public ReefscapeCoralOnField(Pose2d initialPose) {
        super(REEFSCAPE_CORAL_INFO, initialPose);
    }
}
