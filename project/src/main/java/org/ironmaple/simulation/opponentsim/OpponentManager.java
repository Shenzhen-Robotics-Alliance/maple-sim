package org.ironmaple.simulation.opponentsim;

import static edu.wpi.first.units.Units.*;
// TODO

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;

public class OpponentManager {
    // Map of possible scoring poses and types. For example, Map<"Hoops", Map<"CourtLeft", Pose2d>>
    protected static final Map<String, Map<String, Pose2d>> scoringMap = new HashMap<>();
    // Map of possible collecting poses and types. For example, Map<"CollectStation", Map<"StationCenter", Pose2d>>
    protected static final Map<String, Map<String, Pose2d>> collectingMap = new HashMap<>();
    // List of possible starting poses
    protected static final List<Pose2d> initialBluePoses = new ArrayList<>();
    protected static final List<Pose2d> initialRedPoses = new ArrayList<>();
    // Map of queening poses
    protected static final List<Pose2d> queeningPoses = new ArrayList<>();
    // Bounding box buffer, used to determine how big to make the obstacles.
    protected static Distance boundingBoxBuffer;
    // Bounding box offset Translation2d
    protected final Translation2d boundingBoxTranslation;

    /// List of all opponent robots.
    protected static final List<SmartOpponent> opponents = new ArrayList<>();

    /** MapleSim Opponent currently relies on Pathplanner with a modified pathfinder. This is to be changed soon. ^TM */
    public OpponentManager() {
        boundingBoxBuffer = Meters.of(0.6);
        boundingBoxTranslation = new Translation2d(boundingBoxBuffer, boundingBoxBuffer);
    }

    /**
     * MapleSim Opponent currently relies on Pathplanner with a modified pathfinder. This is to be changed soon. ^TM
     *
     * @param withDefaults whether to call withDefaults or not.
     */
    public OpponentManager(boolean withDefaults) {
        this();
        if (withDefaults) {
            withDefaults();
        }
    }

    /**
     * Calls withDefaultQueeningPoses() and withDefaultInitialPoses(). Also warms up pathplanner.
     *
     * @return this, for chaining.
     */
    public OpponentManager withDefaults() {
        PathfindingCommand.warmupCommand().schedule();
        withDefaultInitialPoses().withDefaultQueeningPoses();
        return this;
    }

    /**
     * Adds 6 default queening poses to the list.
     *
     * @return this, for chaining.
     */
    public OpponentManager withDefaultQueeningPoses() {
        /// Add 6 queening poses for 3v3 without any additional
        queeningPoses.add(new Pose2d(-6, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-5, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-4, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-3, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-2, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-1, 0, new Rotation2d()));
        return this;
    }

    /**
     * Add 6 basic starting positions, 3 on each side.
     *
     * @return this, for chaining.
     */
    public OpponentManager withDefaultInitialPoses() {
        /// Add 6 basic starting positions, 3 on each side.
        initialRedPoses.add(new Pose2d(15, 6, Rotation2d.fromDegrees(180)));
        initialRedPoses.add(new Pose2d(15, 4, Rotation2d.fromDegrees(180)));
        initialRedPoses.add(new Pose2d(15, 2, Rotation2d.fromDegrees(180)));
        initialBluePoses.add(new Pose2d(1.6, 6, Rotation2d.kZero));
        initialBluePoses.add(new Pose2d(1.6, 4, Rotation2d.kZero));
        initialBluePoses.add(new Pose2d(1.6, 2, Rotation2d.kZero));
        return this;
    }

    /**
     * Adds an opponent robot to the list of robots to grab things from.
     *
     * @param opponent the {@link SmartOpponent} to manage.
     * @return this, for chaining.
     */
    public OpponentManager registerOpponent(SmartOpponent opponent) {
        opponents.add(opponent);
        return this;
    }

    /**
     * Makes a list of opponents on the given alliance.
     *
     * @param alliance the {@link DriverStation.Alliance} to collect opponents from.
     * @return a {@link List<SmartOpponent>} of the given alliance.
     */
    public List<SmartOpponent> getOpponents(DriverStation.Alliance alliance) {
        List<SmartOpponent> allianceOpponents = new ArrayList<>();
        for (SmartOpponent opponent : opponents) {
            if (opponent.config.alliance == alliance) {
                allianceOpponents.add(opponent);
            }
        }
        return allianceOpponents;
    }

    /**
     * Gets the list of all registered SmartOpponents.
     *
     * @return the {@link List<SmartOpponent>} of registered opponents.
     */
    public List<SmartOpponent> getOpponents() {
        return opponents;
    }

    /**
     * Returns only the opponent targets for the given alliance.
     *
     * @param alliance which {@link DriverStation.Alliance} targets to grab.
     * @return a list of poses targeted by opponents on the given alliance.
     */
    public List<Pair<String, Pose2d>> getOpponentTargets(DriverStation.Alliance alliance) {
        List<Pair<String, Pose2d>> targets = new ArrayList<>();
        for (SmartOpponent opponent : opponents) {
            if (opponent.config.alliance == alliance) {
                targets.add(opponent.getTarget());
            }
        }
        return targets;
    }

    /**
     * Returns all registered opponent target poses.
     *
     * @return a list of all poses targeted by opponents on the given alliance.
     */
    public List<Pair<String, Pose2d>> getOpponentTargets() {
        List<SmartOpponent> allOpponents = getOpponents();
        List<Pair<String, Pose2d>> targets = new ArrayList<>();
        for (SmartOpponent opponent : allOpponents) {
            targets.add(opponent.getTarget());
        }
        return targets;
    }

    /**
     * Returns only the opponent poses for the given alliance.
     *
     * @param alliance which {@link DriverStation.Alliance} poses to grab.
     * @return a list of opponent poses on the given alliance.
     */
    public List<Pose2d> getOpponentPoses(DriverStation.Alliance alliance) {
        List<Pose2d> poses = new ArrayList<>();
        for (SmartOpponent opponent : opponents) {
            if (opponent.config.alliance == alliance) {
                poses.add(opponent.getOpponentPose());
            }
        }
        return poses;
    }

    /**
     * Returns all registered opponent poses.
     *
     * @return a list of all opponents poses on the given alliance.
     */
    public List<Pose2d> getOpponentPoses() {
        List<Pose2d> allPoses = new ArrayList<>();
        for (SmartOpponent opponent : opponents) {
            allPoses.add(opponent.getOpponentPose());
        }
        return allPoses;
    }

    /**
     * Gets a queening pose, removing it from the list.
     *
     * @return a queening pose.
     */
    public Pose2d getQueeningPose() {
        var pose = queeningPoses.get(0);
        queeningPoses.remove(0);
        return pose;
    }

    /**
     * Gets an initial pose, removing it from the list.
     *
     * @return a initial pose.
     */
    public Pose2d getInitialPose(DriverStation.Alliance alliance) {
        Pose2d pose;
        if (alliance == DriverStation.Alliance.Blue) {
            pose = initialBluePoses.get(0);
            initialBluePoses.remove(0);
        } else {
            pose = initialRedPoses.get(0);
            initialRedPoses.remove(0);
        }
        return pose;
    }

    /**
     * Adds a scoring pose.
     *
     * @param poseType The type of pose to add. For example, "Hoops".
     * @param poseName The name of the pose to add. For example, "CourtLeft".
     * @param pose The pose to add.
     * @return this, for chaining.
     */
    public OpponentManager addScoringPose(String poseType, String poseName, Pose2d pose) {
        scoringMap.putIfAbsent(poseType, new HashMap<>());
        scoringMap.get(poseType).putIfAbsent(poseName, pose);
        if (scoringMap.get(poseType).get(poseName) != pose) {
            throw new IllegalArgumentException("Failed to add score pose: " + poseName + "/n");
        }
        return this;
    }

    /**
     * Gets the raw scoring map. Setup as Map<poseType, Map<poseName, Pose2d>>
     *
     * @return the raw scoring map.
     */
    public Map<String, Map<String, Pose2d>> getRawScoringMap() {
        return scoringMap;
    }

    /**
     * Compiles a compacted scoring map. Setup as Map<poseName, Pose2d>
     *
     * @return a compacted scoring map.
     */
    public Map<String, Pose2d> getScoringMap() {
        Map<String, Pose2d> scoringPoses = new HashMap<>();
        scoringMap.forEach((poseType, poseMap) -> poseMap.forEach((name, pose) -> {
            scoringPoses.put(poseType + name, pose);
        }));
        return scoringPoses;
    }

    /**
     * Compiles a list of all scoring poses.
     *
     * @return a list of all scoring poses.
     */
    public List<Pose2d> getScoringPoses() {
        List<Pose2d> scoringPoses = new ArrayList<>();
        scoringMap.forEach((poseType, poseMap) -> poseMap.forEach((poseName, pose) -> scoringPoses.add(pose)));
        return scoringPoses;
    }

    /**
     * Adds a collecting pose to the config.
     *
     * @param poseType The type of pose to add. For example, "CollectStation".
     * @param poseName The name of the pose to add. For example, "StationCenter".
     * @param pose The pose to add.
     * @return this, for chaining.
     */
    public OpponentManager addCollectingPose(String poseType, String poseName, Pose2d pose) {
        collectingMap.putIfAbsent(poseType, new HashMap<>());
        collectingMap.get(poseType).putIfAbsent(poseName, pose);
        if (collectingMap.get(poseType).get(poseName) != pose) {
            throw new IllegalArgumentException("Failed to add collect pose: " + poseName + "/n");
        }
        return this;
    }

    /**
     * Gets the raw collecting map. Setup as Map<poseType, Map<poseName, Pose2d>>
     *
     * @return the raw collecting map.
     */
    public Map<String, Map<String, Pose2d>> getRawCollectingMap() {
        return collectingMap;
    }

    /**
     * Compiles a compacted collecting map. Setup as Map<poseName, Pose2d>
     *
     * @return a compacted collecting map.
     */
    public Map<String, Pose2d> getCollectingMap() {
        Map<String, Pose2d> collectingPoses = new HashMap<>();
        collectingMap.forEach(
                (poseType, poseMap) -> poseMap.forEach((name, pose2d) -> collectingPoses.put(poseType + name, pose2d)));
        return collectingPoses;
    }

    /**
     * Compiles a list of all collecting poses.
     *
     * @return a list of all collecting poses.
     */
    public List<Pose2d> getCollectingPoses() {
        List<Pose2d> collectingPoses = new ArrayList<>();
        collectingMap.forEach((poseType, poseMap) -> poseMap.forEach((poseName, pose) -> collectingPoses.add(pose)));
        return collectingPoses;
    }
}
