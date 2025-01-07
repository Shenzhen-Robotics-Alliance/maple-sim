package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeAlgaeOnFly extends GamePieceProjectile {
    private static Runnable hitNetCallBack;

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
                .withTargetTolerance(new Translation3d(1.22, 3.66, 0.3))
                .withHitTargetCallBack(hitNetCallBack);

        super.withTouchGroundHeight(0.8);
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
