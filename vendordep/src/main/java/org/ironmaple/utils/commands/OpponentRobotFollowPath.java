package org.ironmaple.utils.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * this replaces AutoBuilder because in our code, opponent robots are simulated using path-planner drive commands
 * only the main robot should use AutoBuilder, opponent robots should use this
 * */
public class OpponentRobotFollowPath extends FollowPathHolonomic {
    public OpponentRobotFollowPath(BooleanSupplier shouldFlipPath, HolonomicDriveSubsystem driveSubsystem, double speedMultiplier, Rotation2d endingRotation, Pose2d... poses) {
        this(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(poses),
                        driveSubsystem.getChassisConstrains(speedMultiplier),
                        new GoalEndState(0, endingRotation)
                ),
                shouldFlipPath,
                driveSubsystem
        );
    }

    public OpponentRobotFollowPath(PathPlannerPath path, BooleanSupplier shouldFlipPath, HolonomicDriveSubsystem driveSubsystem) {
        super(
                path,
                driveSubsystem::getPose,
                driveSubsystem::getMeasuredChassisSpeedsRobotRelative,
                driveSubsystem::runRobotCentricChassisSpeeds,
                new PIDConstants(5.0, 0, 0),
                new PIDConstants(5.0, 0, 0),
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(),
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() / driveSubsystem.getChassisMaxAngularVelocity(),
                0.02,
                new ReplanningConfig(false, true),
                shouldFlipPath,
                driveSubsystem
        );
    }

    public static PathPlannerPath reversePath(PathPlannerPath originalPath, GoalEndState endState) {
        final List<PathPoint> newPoints = new ArrayList<>(),
                originalPoints = originalPath.getAllPathPoints();
        for (int i = originalPoints.size()-1; i >= 0; i--) {
            final PathPoint originalPoint = originalPoints.get(i);
            newPoints.add(new PathPoint(
                    originalPoint.position,
                    originalPoint.rotationTarget,
                    originalPoint.constraints
            ));
        }

        return PathPlannerPath.fromPathPoints(newPoints, originalPath.getGlobalConstraints(), endState);
    }
}
