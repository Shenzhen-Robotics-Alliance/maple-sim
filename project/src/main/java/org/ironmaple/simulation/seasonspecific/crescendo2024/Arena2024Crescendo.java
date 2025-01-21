package org.ironmaple.simulation.seasonspecific.crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2024Crescendo extends SimulatedArena {
    /** the obstacles on the 2024 competition field */
    public static final class CrescendoFieldObstaclesMap extends FieldMap {
        private static final double FIELD_WIDTH = 16.54;

        public CrescendoFieldObstaclesMap() {
            super();

            // left wall
            super.addBorderLine(new Translation2d(0, 1), new Translation2d(0, 4.51));
            super.addBorderLine(new Translation2d(0, 4.51), new Translation2d(0.9, 5));

            super.addBorderLine(new Translation2d(0.9, 5), new Translation2d(0.9, 6.05));

            super.addBorderLine(new Translation2d(0.9, 6.05), new Translation2d(0, 6.5));
            super.addBorderLine(new Translation2d(0, 6.5), new Translation2d(0, 8.2));

            // upper wall
            super.addBorderLine(new Translation2d(0, 8.12), new Translation2d(FIELD_WIDTH, 8.12));

            // righter wall
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 1), new Translation2d(FIELD_WIDTH, 4.51));
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 4.51), new Translation2d(FIELD_WIDTH - 0.9, 5));
            super.addBorderLine(new Translation2d(FIELD_WIDTH - 0.9, 5), new Translation2d(FIELD_WIDTH - 0.9, 6.05));
            super.addBorderLine(new Translation2d(FIELD_WIDTH - 0.9, 6.05), new Translation2d(FIELD_WIDTH, 6.5));
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 6.5), new Translation2d(FIELD_WIDTH, 8.2));

            // lower wall
            super.addBorderLine(new Translation2d(1.92, 0), new Translation2d(FIELD_WIDTH - 1.92, 0));

            // red source wall
            super.addBorderLine(new Translation2d(1.92, 0), new Translation2d(0, 1));

            // blue source wall
            super.addBorderLine(new Translation2d(FIELD_WIDTH - 1.92, 0), new Translation2d(FIELD_WIDTH, 1));

            // blue state
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(3.4, 4.1, new Rotation2d()));
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(5.62, 4.1 - 1.28, Rotation2d.fromDegrees(30)));
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(5.62, 4.1 + 1.28, Rotation2d.fromDegrees(60)));

            // red stage
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(FIELD_WIDTH - 3.4, 4.1, new Rotation2d()));
            super.addRectangularObstacle(
                    0.35, 0.35, new Pose2d(FIELD_WIDTH - 5.62, 4.1 - 1.28, Rotation2d.fromDegrees(60)));
            super.addRectangularObstacle(
                    0.35, 0.35, new Pose2d(FIELD_WIDTH - 5.62, 4.1 + 1.28, Rotation2d.fromDegrees(30)));
        }
    }

    private static final Translation2d[] NOTE_INITIAL_POSITIONS = new Translation2d[] {
        new Translation2d(2.9, 4.1),
        new Translation2d(2.9, 5.55),
        new Translation2d(2.9, 7),
        new Translation2d(8.27, 0.75),
        new Translation2d(8.27, 2.43),
        new Translation2d(8.27, 4.1),
        new Translation2d(8.27, 5.78),
        new Translation2d(8.27, 7.46),
        new Translation2d(13.64, 4.1),
        new Translation2d(13.64, 5.55),
        new Translation2d(13.64, 7),
    };

    public Arena2024Crescendo() {
        super(new CrescendoFieldObstaclesMap());
    }

    @Override
    public void placeGamePiecesOnField() {
        for (Translation2d notePosition : NOTE_INITIAL_POSITIONS)
            super.addGamePiece(new CrescendoNoteOnField(notePosition));

        super.addCustomSimulation(new CrescendoHumanPlayerSimulation(this));
    }
}
