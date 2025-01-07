package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

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
    private static Runnable hitNetCallBack = () -> System.out.println("hit target!");

    /**
     *
     *
     * <h2>Specifies a callback for when any ALGAE launched into the air hits the NET.</h2>
     *
     * @param callBack a {@link Runnable} to be invoked when an ALGAE hits the NET
     */
    public static void setHitNetCallBack(Runnable callBack) {
        hitNetCallBack = callBack;
    }

    public ReefscapeAlgaeOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            double initialHeight,
            double launchingSpeedMPS,
            double shooterAngleRad) {
        super(
                ReefscapeAlgaeOnField.REEFSCAPE_ALGAE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeedMPS,
                shooterAngleRad);

        super.withTargetPosition(
                        () -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(8.785, 1.906, 2.1)))
                .withTargetTolerance(new Translation3d(0.8, 3, 0.1))
                .withHitTargetCallBack(hitNetCallBack);

        super.withTouchGroundHeight(0.8);
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
