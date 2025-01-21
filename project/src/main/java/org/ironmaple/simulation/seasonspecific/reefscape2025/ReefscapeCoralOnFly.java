package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class ReefscapeCoralOnFly extends GamePieceProjectile {
    public ReefscapeCoralOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(
                ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
        super.withTouchGroundHeight(0.2);
    }

    @Override
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
        simulatedArena.addGamePiece(new GamePieceOnFieldSimulation(
                ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                () -> Math.max(
                        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO
                                        .gamePieceHeight()
                                        .in(Meters)
                                / 2,
                        getPositionAtTime(super.launchedTimer.get()).getZ()),
                new Pose2d(
                        getPositionAtTime(launchedTimer.get()).toTranslation2d(),
                        initialLaunchingVelocityMPS.getAngle()),
                super.initialLaunchingVelocityMPS));
    }
}
