package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import java.util.Arrays;
import java.util.List;

public class ReefAlignment {
    public static final Distance ALIGNMENT_TARGET_DISTANCE = Centimeters.of(45);
    public static final Distance ALIGNMENT_TARGET_STRAFE = Centimeters.of(16);
    public static final PathConstraints alignmentConstrains = new PathConstraints(
            MetersPerSecond.of(2.5),
            MetersPerSecondPerSecond.of(4.0),
            DegreesPerSecond.of(180),
            DegreesPerSecondPerSecond.of(360),
            Volts.of(12));

    public static List<Pose2d> REEF_TAG_POSES;

    private final AprilTagFieldLayout tagsLayout;
    private final Drive drive;

    public ReefAlignment(Drive drive) {
        this.drive = drive;
        this.tagsLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        int[] reefTagIds = new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        REEF_TAG_POSES = Arrays.stream(reefTagIds).mapToObj(this::getTag2dPose).toList();
    }

    public enum AlignmentSide {
        LEFT,
        RIGHT,
        CENTER
    }

    public Command alignToNearestReef(AlignmentSide side) {
        return Commands.deferredProxy(
                () -> drive.pathFindTo(getAppropriateAlignmentTargetPose(side), alignmentConstrains));
    }

    private Pose2d getAppropriateAlignmentTargetPose(AlignmentSide side) {
        Pose2d currentPose = drive.getPose();
        Pose2d nearestTagPose = currentPose.nearest(REEF_TAG_POSES);

        double strafeFactor =
                switch (side) {
                    case LEFT -> -1.0;
                    case RIGHT -> 1.0;
                    case CENTER -> 0.0;
                };

        Twist2d tagPoseToAlignmentTargetPosition = new Twist2d(
                ALIGNMENT_TARGET_DISTANCE.in(Meters), ALIGNMENT_TARGET_STRAFE.in(Meters) * strafeFactor, 0.0);
        Translation2d alignmentTargetPosition =
                nearestTagPose.exp(tagPoseToAlignmentTargetPosition).getTranslation();
        Rotation2d alignmentTargetRotation = nearestTagPose.getRotation().rotateBy(Rotation2d.k180deg);

        Pose2d alignmentTarget = new Pose2d(alignmentTargetPosition, alignmentTargetRotation);
        DogLog.log("ReefAlignment/AlignmentTarget", alignmentTarget);
        return alignmentTarget;
    }

    private static final Pose2d nullPose = new Pose2d(999, 999, new Rotation2d());

    private Pose2d getTag2dPose(int tagID) {
        return tagsLayout.getTagPose(tagID).map(Pose3d::toPose2d).orElse(nullPose);
    }
}
