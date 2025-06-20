package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
/**
 *
 *
 * <h1>Interface for all Gamee pieces.</h1>
 *
 * <p>This class contains baisc functions all game peices will have so that game peices of different types can be used interchangably in collision detection.
 */
public interface GamePieceInterface {

    /**
     * <h2> gives the pose3d of a game piece.</h2>
     * @return the pose of this piece as a Pose3d.
     */
    public Pose3d getPose3d();

    /**
     * <h2> gives the string type of the current game piece.</h2> 
     * @return the game peice string type (ex "Coral", "Algae", "Note").
     */
    public String getType();

    /**
     * <h2> Gives the velocity of the game piece.</h2>
     * 
     * For grounded game pieces the z access velocity does not exist and so will be set to 0 automaticly.
     * 
     * @return The velocity of the game piece as a Translation3d.
     */
    public Translation3d getVelocity3dMPS();


    /**
     * <h2>Gives wether or not the peice is "grounded"</h2> 
     * 
     * 
     * A grounded piece is likly a child of {@link GamePieceOnFieldSimulation} while a non grounded piece is likly a child of{@link GamePieceProjectile}
     * @return wether or not the peice is grounded 
     */
    public boolean isGrounded();
}
