package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 *
 *
 * <h1>Interface for all Game pieces.</h1>
 *
 * <p>This class contains basic functions all game pieces will have so that game pieces of different types can be used
 * interchangeably in collision detection.
 */
public interface GamePiece {

    /**
     *
     *
     * <h2>Gives the pose3d of a game piece.</h2>
     *
     * @return The pose of this piece as a Pose3d.
     */
    Pose3d getPose3d();

    /**
     *
     *
     * <h2>Gives the string type of the current game piece.</h2>
     *
     * @return The game piece string type (ex "Coral", "Algae", "Note").
     */
    String getType();

    /**
     *
     *
     * <h2>Gives the velocity of the game piece.</h2>
     *
     * For grounded game pieces the z access velocity does not exist and so will be set to 0 automatically.
     *
     * @return The velocity of the game piece as a Translation3d.
     */
    Translation3d getVelocity3dMPS();

    /**
     *
     *
     * <h2>Gives wether or not the piece is "grounded".</h2>
     *
     * A grounded piece is likely a child of {@link GamePieceOnFieldSimulation} while a non grounded piece is likely a
     * child of{@link GamePieceProjectile}.
     *
     * @return wether or not the piece is grounded.
     */
    boolean isGrounded();
}
