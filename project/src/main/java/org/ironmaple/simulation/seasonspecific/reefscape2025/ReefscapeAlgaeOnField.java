package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 *
 *
 * <h1>Represents an ALGAE in the 2025 Reefscape game.</h1>
 *
 * <p>The Algae (<a href="https://www.andymark.com/products/frc-2024?sku=am-5602">am-5602</a>) is a 16-inch (41 cm)
 * rubber playground ball with a ±½ inch (~12 mm) diameter, featured as a game piece in the 2025 Reefscape game.
 */
public class ReefscapeAlgaeOnField extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo REEFSCAPE_ALGAE_INFO =
            new GamePieceInfo("Algae", new Circle(0.176), Inches.of(16), Kilograms.of(0.4), 1.8, 5, 0.8);

    public ReefscapeAlgaeOnField(Translation2d initialPosition) {
        super(REEFSCAPE_ALGAE_INFO, new Pose2d(initialPosition, new Rotation2d()));
    }
}
