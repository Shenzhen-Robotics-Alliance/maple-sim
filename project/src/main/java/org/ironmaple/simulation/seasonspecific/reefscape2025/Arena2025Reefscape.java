package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2025Reefscape extends SimulatedArena {
    public static final class ReefscapeFieldObstacleMap extends FieldMap {
        public ReefscapeFieldObstacleMap() {
            super();

            // blue wall
            super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

            // blue coral stations
            super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672, 0));
            super.addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052));

            // red wall
            super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

            // red coral stations
            super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548-1.672, 0));
            super.addBorderLine(new Translation2d(17.548, 6.782), new Translation2d(17.548-1.672, 8.052));

            // upper walls
            super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(17.548-1.672, 8.052));

            // lower walls
            super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(17.548-1.672, 0));
        }
    }

    public Arena2025Reefscape() {
        super(new ReefscapeFieldObstacleMap());
    }

    @Override
    public void placeGamePiecesOnField() {

    }

    @Override
    public void competitionPeriodic() {

    }
}
