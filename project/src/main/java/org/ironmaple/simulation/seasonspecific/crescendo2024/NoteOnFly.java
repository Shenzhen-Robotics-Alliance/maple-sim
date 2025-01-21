package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.LegacyFieldMirroringUtils2024;

public class NoteOnFly extends GamePieceProjectile {
    public NoteOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(
                CRESCENDO_NOTE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);
        super.withTouchGroundHeight(0.6);
    }

    public NoteOnFly asSpeakerShotNote(Runnable hitTargetCallBack) {
        return (NoteOnFly) super.withTargetPosition(() ->
                        LegacyFieldMirroringUtils2024.toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
                .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
                .withHitTargetCallBack(hitTargetCallBack);
    }

    public NoteOnFly asAmpShotNote(Runnable hitTargetCallBack) {
        return (NoteOnFly) super.withTargetPosition(() ->
                        LegacyFieldMirroringUtils2024.toCurrentAllianceTranslation(new Translation3d(1.83, 8.12, 0.95)))
                .withTargetTolerance(new Translation3d(0.1, 0.6, 0.45))
                .withHitTargetCallBack(hitTargetCallBack);
    }

    public NoteOnFly enableBecomeNoteOnFieldAfterTouchGround() {
        return (NoteOnFly) super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }

    public NoteOnFly disableBecomeNoteOnFieldAfterTouchGround() {
        return (NoteOnFly) super.disableBecomesGamePieceOnFieldAfterTouchGround();
    }
}
