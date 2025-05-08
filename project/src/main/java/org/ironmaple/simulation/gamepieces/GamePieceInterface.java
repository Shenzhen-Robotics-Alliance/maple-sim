package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public interface GamePieceInterface {
    public Pose3d getPose3d();

    public String getType();

    public Translation3d getVelocity3dMPS();

    public boolean isGrounded();
}
