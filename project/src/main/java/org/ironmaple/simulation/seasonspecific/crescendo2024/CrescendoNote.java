package org.ironmaple.simulation.seasonspecific.crescendo2024;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.List;

import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.GamePiece.GamePieceVariant;

public class CrescendoNote {
  /* https://www.andymark.com/products/frc-2024-am-4999 */
  private static final double NOTE_HEIGHT = Units.inchesToMeters(2),
      NOTE_DIAMETER = Units.inchesToMeters(14),
      NOTE_WEIGHT_KG = 0.2;

  public static final GamePieceVariant VARIANT = new GamePieceVariant(
      "Note",
      NOTE_HEIGHT,
      NOTE_WEIGHT_KG,
      Geometry.createCircle(NOTE_DIAMETER / 2),
      List.of(
        new Pair<>(
          new Translation3d(-0.25, 4.36, 2.0),
          new Translation3d(0.75, 6.76, 2.6)
        ),
        new Pair<>(
          new Translation3d(15.8, 4.36, 2.0),
          new Translation3d(16.8, 6.76, 2.6)
        )
      ),
      true
  );
}
