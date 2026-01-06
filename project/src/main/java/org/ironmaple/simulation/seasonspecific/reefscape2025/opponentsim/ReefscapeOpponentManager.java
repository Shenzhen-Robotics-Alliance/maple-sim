package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.opponentsim.OpponentManager;

public class ReefscapeOpponentManager extends OpponentManager {
    public ReefscapeOpponentManager() {
        super(true); // Default Starting and queening poses.
        super
                /// All reef scoring poses
                .addScoringPose("Reef", "Reef South Left", PoseConstants.REEF_SOUTH_LEFT_POSE)
                .addScoringPose("Reef", "Reef South  Right", PoseConstants.REEF_SOUTH_RIGHT_POSE)
                .addScoringPose("Reef", "Reef Southeast Left", PoseConstants.REEF_SOUTHEAST_LEFT_POSE)
                .addScoringPose("Reef", "Reef Southeast Right", PoseConstants.REEF_SOUTHEAST_RIGHT_POSE)
                .addScoringPose("Reef", "Reef Northeast Left", PoseConstants.REEF_NORTHEAST_LEFT_POSE)
                .addScoringPose("Reef", "Reef Northeast Right", PoseConstants.REEF_NORTHEAST_RIGHT_POSE)
                .addScoringPose("Reef", "Reef North Left", PoseConstants.REEF_NORTH_LEFT_POSE)
                .addScoringPose("Reef", "Reef North Right", PoseConstants.REEF_NORTH_RIGHT_POSE)
                .addScoringPose("Reef", "Reef Northwest Left", PoseConstants.REEF_NORTHWEST_LEFT_POSE)
                .addScoringPose("Reef", "Reef Northwest Right", PoseConstants.REEF_NORTHWEST_RIGHT_POSE)
                .addScoringPose("Reef", "Reef Southwest Left", PoseConstants.REEF_SOUTHWEST_LEFT_POSE)
                .addScoringPose("Reef", "Reef Southwest Right", PoseConstants.REEF_SOUTHWEST_RIGHT_POSE)
                /// Barge scoring pose
                .addScoringPose("Barge", "Net", PoseConstants.BARGE_NET_POSE)
                /// Coral Station Poses
                .addCollectingPose(
                        "Left Station", "Left", PoseConstants.LEFT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_LEFT))
                .addCollectingPose(
                        "Left Station", "Center", PoseConstants.LEFT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_RIGHT))
                .addCollectingPose(
                        "Left Station", "Right", PoseConstants.LEFT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_RIGHT))
                .addCollectingPose(
                        "Right Station", "Left", PoseConstants.RIGHT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_LEFT))
                .addCollectingPose("Right Station", "Center", PoseConstants.RIGHT_STATION_POSE)
                .addCollectingPose(
                        "Right Station",
                        "Right",
                        PoseConstants.RIGHT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_RIGHT));
    }

    /// Its end of the season, so I just copied my constants here and added them as such.
    /// Typically, it would probably just be a new Pose2d()...
    private static class PoseConstants {
        public static final Transform2d STATION_OFFSET =
                new Transform2d(Units.inchesToMeters(-17), Units.inchesToMeters(0), Rotation2d.kZero);
        public static final Transform2d SLOT_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(24), // Added several times to achieve all 5 poses.
                Rotation2d.kZero);
        public static final Transform2d SLOT_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(-30), // Added several times to achieve all 5 poses.
                Rotation2d.kZero);
        public static final Pose2d BARGE_NET_POSE = new Pose2d(7.5, 2, Rotation2d.fromDegrees(0));
        public static final Pose2d LEFT_STATION_CENTER_POSE = new Pose2d(
                Units.inchesToMeters(33.526), Units.inchesToMeters(291.176), Rotation2d.fromDegrees(125.989));
        public static final Pose2d RIGHT_STATION_CENTER_POSE =
                new Pose2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824), Rotation2d.fromDegrees(234.011));
        public static final Pose2d LEFT_STATION_POSE = LEFT_STATION_CENTER_POSE.plus(STATION_OFFSET);
        public static final Pose2d RIGHT_STATION_POSE = RIGHT_STATION_CENTER_POSE.plus(STATION_OFFSET);
        public static final Pose2d SOUTH_FACE_POSE =
                new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(0));
        public static final Pose2d SOUTHWEST_FACE_POSE =
                new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(300));
        public static final Pose2d NORTHWEST_FACE_POSE =
                new Pose2d(Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(240));
        public static final Pose2d NORTH_FACE_POSE =
                new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(180));
        public static final Pose2d NORTHEAST_FACE_POSE =
                new Pose2d(Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(120));
        public static final Pose2d SOUTHEAST_FACE_POSE =
                new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(60));
        public static final Transform2d BRANCH_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from the reef.
                Units.inchesToMeters(13 / 2.0), // Offset to left branch.
                Rotation2d.kZero);
        public static final Transform2d BRANCH_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from the reef.
                Units.inchesToMeters(-13 / 2.0), // Offset to the right branch.
                Rotation2d.kZero);
        public static final Pose2d REEF_NORTH_LEFT_POSE = NORTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTH_RIGHT_POSE = NORTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHEAST_LEFT_POSE = NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHEAST_RIGHT_POSE = NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHWEST_LEFT_POSE = NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHWEST_RIGHT_POSE = NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTH_LEFT_POSE = SOUTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTH_RIGHT_POSE = SOUTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHEAST_LEFT_POSE = SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHEAST_RIGHT_POSE = SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHWEST_LEFT_POSE = SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHWEST_RIGHT_POSE = SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
    }
}
