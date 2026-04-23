package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 *
 *
 * <h1>Represents an fuel in the 2026 Rebuilt game.</h1>
 */
public class RebuiltFuelOnField extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo REBUILT_FUEL_INFO = new GamePieceInfo(
            "Fuel", new Circle(Centimeters.of(7.5).in(Meters)), Centimeter.of(15), Pounds.of(0.5), 1.8, 5, 0.8);

    public RebuiltFuelOnField(Translation2d initialPosition) {
        super(REBUILT_FUEL_INFO, new Pose2d(initialPosition, new Rotation2d()));
    }
}
