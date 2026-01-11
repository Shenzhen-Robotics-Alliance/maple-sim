package org.ironmaple.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class FieldMirroringUtils {
    public static final double FIELD_WIDTH = 17.548;
    public static final double FIELD_HEIGHT = 8.052;

    public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
        return isSidePresentedAsRed() ? flip(rotationAtBlueSide) : rotationAtBlueSide;
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.plus(Rotation2d.k180deg);
    }

    public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
        return isSidePresentedAsRed() ? flip(translationAtBlueSide) : translationAtBlueSide;
    }

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(FIELD_WIDTH - translation.getX(), FIELD_HEIGHT - translation.getY());
    }

    public static Pose3d flip(Pose3d toFlip) {
        return new Pose3d(
                new Translation3d(FIELD_WIDTH - toFlip.getX(), FIELD_HEIGHT - toFlip.getY(), toFlip.getZ()),
                toFlip.getRotation());
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
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
    }

    public static Rotation2d getCurrentAllianceDriverStationFacing() {
        return toCurrentAllianceRotation(Rotation2d.kZero);
    }
}
