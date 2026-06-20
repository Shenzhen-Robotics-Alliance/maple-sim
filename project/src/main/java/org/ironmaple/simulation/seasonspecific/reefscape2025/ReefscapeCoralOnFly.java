package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static org.wpilib.units.Units.*;

import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeCoralOnFly extends GamePieceProjectile {
    public ReefscapeCoralOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisVelocities chassisSpeeds,
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

    public enum CoralStationsSide {
        LEFT_STATION(new Pose2d(0.89, 7.32, Rotation2d.fromDegrees(-54))),
        RIGHT_STATION(new Pose2d(0.89, 0.6, Rotation2d.fromDegrees(54)));

        private final Pose2d startingPose;

        CoralStationsSide(Pose2d startingPose) {
            this.startingPose = startingPose;
        }
    }

    public static ReefscapeCoralOnFly DropFromCoralStation(
            CoralStationsSide station, Alliance alliance, boolean isHorizontal) {
        Rotation2d rot = alliance == Alliance.RED
                ? FieldMirroringUtils.flip(station.startingPose.getRotation())
                : station.startingPose.getRotation();
        Translation2d pos = alliance == Alliance.RED
                ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
                : station.startingPose.getTranslation();
        return isHorizontal
                ? new ReefscapeCoralOnFly(
                        pos,
                        new Translation2d(),
                        new ChassisVelocities(3.0, 0, 0).toFieldRelative(rot),
                        rot.rotateBy(Rotation2d.kCCW_90deg),
                        Centimeters.of(98),
                        MetersPerSecond.of(0),
                        Degrees.of(0))
                : new ReefscapeCoralOnFly(
                        alliance == Alliance.RED
                                ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
                                : station.startingPose.getTranslation(),
                        new Translation2d(),
                        new ChassisVelocities(),
                        alliance == Alliance.RED
                                ? FieldMirroringUtils.flip(station.startingPose.getRotation())
                                : station.startingPose.getRotation(),
                        Centimeters.of(98),
                        MetersPerSecond.of(3),
                        Degrees.of(-50));
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
