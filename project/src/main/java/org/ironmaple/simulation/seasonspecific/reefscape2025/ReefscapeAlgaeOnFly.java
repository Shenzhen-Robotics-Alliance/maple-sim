package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/**
 *
 *
 * <h1>Represents an ALGAE launched into the air.</h1>
 *
 * <p>This class models a {@link ReefscapeAlgaeOnField} launched into the air.
 *
 * <p>The simulation will determine if the ALGAE hits its target—the NET.
 *
 * <p>The user can specify a callback using {@link #setHitNetCallBack(Runnable)}, which will be triggered when the ALGAE
 * hits the NET.
 */
public class ReefscapeAlgaeOnFly extends GamePieceProjectile {

    public ReefscapeAlgaeOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(
                ReefscapeAlgaeOnField.REEFSCAPE_ALGAE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);

        super.withTouchGroundHeight(0.8);
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
