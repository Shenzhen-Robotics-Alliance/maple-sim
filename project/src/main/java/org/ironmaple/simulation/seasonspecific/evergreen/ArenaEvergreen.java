package org.ironmaple.simulation.seasonspecific.evergreen;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.SimulatedArena;

/**
 *
 *
 * <h1>A field with no obstacles </h1>
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
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, 8));
            // red wall
            super.addBorderLine(new Translation2d(17.548, 0), new Translation2d(17.548, 8));
            // upper wall
            super.addBorderLine(new Translation2d(0, 8.052), new Translation2d(20, 8.052));
            // lower wall
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(20, 0));
            return this;
        }
    }

    /**
     *
     *
     * <h1>A field with no obstacles </h1>
     *
     * <p>This class represents the playing field for the simulated Evergreen Field
     *
     * <p>It extends {@link SimulatedArena}.
     *
     * @param withWalls used to add wall obstacles when desired.
     */
	public ArenaEvergreen(boolean withWalls) {
        super(withWalls ? new EvergreenFieldObstacleMap().withWalls() : new EvergreenFieldObstacleMap());
    }

    /**
     *
     *
     * <h2>Evergreen does not have game pieces, this method does nothing. </h2>
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
