package org.ironmaple.simulation.seasonspecific.crescendo2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

import static org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField.*;

public class NoteOnFly extends GamePieceProjectile {
    public NoteOnFly(Translation2d robotPosition, Translation2d shooterPositionOnRobot, ChassisSpeeds chassisSpeeds, Rotation2d shooterFacing, double initialHeight, double launchingSpeedMPS, double shooterAngleRad) {
        super("Note", robotPosition, shooterPositionOnRobot, chassisSpeeds, shooterFacing, initialHeight, launchingSpeedMPS, shooterAngleRad);
        super.enableBecomesGamePieceOnFieldAfterTouchGround(
                Geometry.createCircle(NOTE_DIAMETER/2),
                NOTE_HEIGHT,
                NOTE_WEIGHT_KG
        ).withTouchGroundHeight(0.6);
    }

    public NoteOnFly asSpeakerShotNote(Runnable hitTargetCallBack) {
        return (NoteOnFly) super.withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
                .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
                .withHitTargetCallBack(hitTargetCallBack)
                .disableBecomesGamePieceOnFieldAfterTouchGround();
    }

    public NoteOnFly asAmpShotNote(Runnable hitTargetCallBack) {
        return (NoteOnFly) super.withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(1.83, 8.12, 0.95)))
                .withTargetTolerance(new Translation3d(0.1, 0.6, 0.45))
                .withHitTargetCallBack(hitTargetCallBack);
    }
}
