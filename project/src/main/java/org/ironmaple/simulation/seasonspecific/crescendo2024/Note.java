package org.ironmaple.simulation.seasonspecific.crescendo2024;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.List;

import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.GamePiece.GamePieceTarget;
import org.ironmaple.simulation.GamePiece.GamePieceVariant;

public class Note {
  /** https://www.andymark.com/products/frc-2024-am-4999 */
  public static final GamePieceVariant VARIANT = new GamePieceVariant(
      "Note",
      Units.inchesToMeters(2),
      0.2,
      Geometry.createCircle(Units.inchesToMeters(14) / 2),
      List.of(
        new GamePieceTarget(
          new Translation3d(-0.25, 4.36, 2.0),
          new Translation3d(0.75, 6.76, 2.6)
        ),
        new GamePieceTarget(
          new Translation3d(15.8, 4.36, 2.0),
          new Translation3d(16.8, 6.76, 2.6)
        )
      ),
      true,
      0.2
  );
}
