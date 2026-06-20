package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class CrescendoNoteOnField extends GamePieceOnFieldSimulation {
    /* https://www.andymark.com/products/frc-2024-am-4999 */
    public static final GamePieceInfo CRESCENDO_NOTE_INFO = new GamePieceInfo(
            "Note", Geometry.createCircle(Units.inchesToMeters(14) / 2), Inches.of(2), Kilograms.of(0.2), 3.5, 5, 0.3);

    public CrescendoNoteOnField(Translation2d initialPosition) {
        super(CRESCENDO_NOTE_INFO, new Pose2d(initialPosition, new Rotation2d()));
    }
}
