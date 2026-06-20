package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField.*;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.LegacyFieldMirroringUtils2024;

public class NoteOnFly extends GamePieceProjectile {
    public NoteOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisVelocities chassisVelocities,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingVelocity,
            Angle shooterAngle) {
        super(
                CRESCENDO_NOTE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisVelocities,
                shooterFacing,
                initialHeight,
                launchingVelocity,
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
