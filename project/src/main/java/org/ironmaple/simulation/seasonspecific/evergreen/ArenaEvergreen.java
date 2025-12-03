package org.ironmaple.simulation.seasonspecific.evergreen;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.SimulatedArena;

/**
 * <h1> A field with no obstacles </h1>
 *
 * <p>This class represents the playing field for the simulated Evergreen Field
 *
 * <p>It extends {@link SimulatedArena}.
 */
public class ArenaEvergreen extends SimulatedArena {
    public static final class EvergreenFieldObstacleMap extends FieldMap {
        public EvergreenFieldObstacleMap() {
            super();
        }

        /**
         * Used for adding walls when desired.
         *
         * @return this, for chaining.
         */
        public EvergreenFieldObstacleMap withWalls() {
            // blue wall
            super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

            // red wall
            super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

            // upper walls
            super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(11, 8.052));
            super.addBorderLine(new Translation2d(12, 8.052), new Translation2d(17.548 - 1.672, 8.052));

            // lower walls
            super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(5.8, 0));
            super.addBorderLine(new Translation2d(6.3, 0), new Translation2d(17.548 - 1.672, 0));
            return this;
        }
    }

    /**
     * <h1> A field with no obstacles </h1>
     *
     * <p>This class represents the playing field for the simulated Evergreen Field
     *
     * <p>It extends {@link SimulatedArena}.
     *
     * @param withWalls used to add wall obstacles when desired.
     */
	public ArenaEvergreen(boolean withWalls) {
        super(withWalls ? new EvergreenFieldObstacleMap() : new EvergreenFieldObstacleMap().withWalls());
    }

    /**
     * <h2> Evergreen does not have game pieces, this method does nothing. </h2>
     *
     * <p>Places Game Pieces on the Field for Autonomous Mode.
     *
     * <p>This method sets up the game pieces on the field, typically in their starting positions for autonomous mode.
     *
     * <p>It should be implemented differently for each season-specific subclass of {@link SimulatedArena} to reflect
     * the unique game piece placements for that season's game.
     */
    @Override
    public void placeGamePiecesOnField() {
        DriverStation.reportError("Evergreen doesn't have game pieces.", false);
    }
}
