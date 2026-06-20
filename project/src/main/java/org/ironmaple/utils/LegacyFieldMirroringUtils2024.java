package org.ironmaple.utils;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;

import java.util.Optional;
import java.util.function.Supplier;

public class LegacyFieldMirroringUtils2024 {
    public static final double FIELD_WIDTH = 16.54;
    public static final double FIELD_HEIGHT = 8.21;

    public static final Translation3d SPEAKER_POSE_BLUE = new Translation3d(0.1, 5.55, 2.2);

    public static final Supplier<Translation2d> SPEAKER_POSITION_SUPPLIER =
            () -> toCurrentAllianceTranslation(SPEAKER_POSE_BLUE.toTranslation2d());

    public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
        final Rotation2d yAxis = Rotation2d.fromDegrees(90),
                differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis),
                differenceFromYAxisNew = differenceFromYAxisAtBlueSide.times(isSidePresentedAsRed() ? -1 : 1);
        return yAxis.rotateBy(differenceFromYAxisNew);
    }

    public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
        if (isSidePresentedAsRed())
            return new Translation2d(FIELD_WIDTH - translationAtBlueSide.getX(), translationAtBlueSide.getY());
        return translationAtBlueSide;
    }

    public static Translation3d toCurrentAllianceTranslation(Translation3d translation3dAtBlueSide) {
        final Translation2d translation3dAtCurrentAlliance =
                toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d());
        if (isSidePresentedAsRed())
            return new Translation3d(
                    translation3dAtCurrentAlliance.getX(),
                    translation3dAtCurrentAlliance.getY(),
                    translation3dAtBlueSide.getZ());
        return translation3dAtBlueSide;
    }

    public static Pose2d toCurrentAlliancePose(Pose2d poseAtBlueSide) {
        return new Pose2d(
                toCurrentAllianceTranslation(poseAtBlueSide.getTranslation()),
                toCurrentAllianceRotation(poseAtBlueSide.getRotation()));
    }

    public static boolean isSidePresentedAsRed() {
        final Optional<Alliance> alliance = MatchState.getAlliance();
        return alliance.isPresent() && alliance.get().equals(Alliance.RED);
    }

    public static Rotation2d getCurrentAllianceDriverStationFacing() {
        return switch (MatchState.getAlliance().orElse(Alliance.RED)) {
            case RED -> new Rotation2d(Math.PI);
            case BLUE -> new Rotation2d(0);
        };
    }
}
