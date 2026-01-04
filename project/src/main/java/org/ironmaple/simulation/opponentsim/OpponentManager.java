package org.ironmaple.simulation.opponentsim;

import static edu.wpi.first.units.Units.*;
// TODO

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.opponentsim.pathfinding.MapleADStar;

import java.lang.reflect.Parameter;
import java.util.*;

public class OpponentManager {
    // Map of possible scoring poses and types. For example, Map<"Hoops", Map<"CourtLeft", Pose2d>>
    protected static final Map<String, Map<String, Pose2d>> scoringMap = new HashMap<>();
    // Map of possible collecting poses and types. For example, Map<"CollectStation", Map<"StationCenter", Pose2d>>
    protected static final Map<String, Map<String, Pose2d>> collectingMap = new HashMap<>();
    /// Cached opponent data for dynamic polling.
    // Cached list of all opponent obstacles.
    protected static final List<Pair<Translation2d, Translation2d>> opponentObstacles = new ArrayList<>();
    protected double lastObstaclePoll = 0;
    protected static final List<Pair<Translation2d, Translation2d>> blueOpponentObstacles = new ArrayList<>();
    protected double lastBlueObstaclePoll = 0;
    protected static final List<Pair<Translation2d, Translation2d>> redOpponentObstacles = new ArrayList<>();
    protected double lastRedObstaclePoll = 0;
    // Cached list of all opponent targets.
    protected static final List<Pair<String, Pose2d>> opponentTargets = new ArrayList<>();
    protected double lastTargetPoll = 0;
    protected static final List<Pair<String, Pose2d>> blueOpponentTargets = new ArrayList<>();
    protected double lastBlueTargetPoll = 0;
    protected static final List<Pair<String, Pose2d>> redOpponentTargets = new ArrayList<>();
    protected double lastRedTargetPoll = 0;
    // Cached list of all opponent poses.
    protected static final List<Pose2d> opponentPoses = new ArrayList<>();
    protected double lastPosePoll = 0;
    protected static final List<Pose2d> blueOpponentPoses = new ArrayList<>();
    protected double lastBluePosePoll = 0;
    protected static final List<Pose2d> redOpponentPoses = new ArrayList<>();
    protected double lastRedPosePoll = 0;
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
        this.boundingBoxTranslation = new Translation2d(boundingBoxBuffer, boundingBoxBuffer);
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
        this
                .withDefaultInitialPoses()
                .withDefaultQueeningPoses();
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
    public List<Pair<String, Pose2d>> getOpponentTargetsDynamic(DriverStation.Alliance alliance, Time pollRate) {
        // If elapsed time is less than pollRate return our cached list.
        final var isBlue = alliance == DriverStation.Alliance.Blue;
        final var targetList = isBlue ? blueOpponentTargets : redOpponentTargets;
        if (System.currentTimeMillis() - (isBlue ? lastBlueTargetPoll : lastRedTargetPoll) < pollRate.in(Milliseconds)) {
            return targetList;
        }
        // New list to filter.
        final var filteredOpponents = new ArrayList<>(opponents);
        // If elapsed time exceeds our pollRate, refresh the list.
        targetList.clear();
        filteredOpponents.stream()
                // Remove all opponents not on given alliance.
                .filter(opponent -> opponent.config.alliance != alliance)
                // Collect all their poses.
                .forEach(opponent -> targetList.add(opponent.getTarget()));
        // Update our refresh timestamp.
        if (isBlue) {
            lastBlueTargetPoll = System.currentTimeMillis();
        } else {
            lastRedTargetPoll = System.currentTimeMillis();
        }
        return targetList;
    }

    /**
     * TODO
     * @param pollRate
     * @return
     */
    public List<Pair<String, Pose2d>> getOpponentTargetsDynamic(Time pollRate)
    {
        // If elapsed time is less than pollRate return our cached list.
        if (System.currentTimeMillis() - lastTargetPoll < pollRate.in(Milliseconds)) {
            return opponentTargets;
        }
        // If elapsed time exceeds our pollRate, refresh the list.
        opponentTargets.clear();
        // Call the other dynamic methods so we don't update if not needed.
        opponentTargets.addAll(getOpponentTargetsDynamic(DriverStation.Alliance.Blue, pollRate));
        opponentTargets.addAll(getOpponentTargetsDynamic(DriverStation.Alliance.Red, pollRate));
        // Update our refresh timestamp.
        lastTargetPoll = System.currentTimeMillis();
        return opponentTargets;
    }

    /**
     * Checks if the given pose is near any opponent target pose.
     *
     * @param pose the pose to check against.
     * @param pollRate TODO
     * @param tolerance the translation tolerance in {@link Distance}.
     * @return
     */
    public boolean isNearTarget(Pose2d pose, Time pollRate, Distance tolerance) {
        for (Pair<String, Pose2d> existingTarget : getOpponentTargetsDynamic(pollRate)) {
            // Check if the new target is within the tolerance distance of any existing target
            if (Objects.nonNull(existingTarget)) {
                if (existingTarget.getSecond().getTranslation().getDistance(pose.getTranslation()) < tolerance.in(Meters)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Returns only the opponent poses for the given alliance.
     *
     * @param alliance which {@link DriverStation.Alliance} poses to grab.
     * @return a list of opponent poses on the given alliance.
     */
    public List<Pose2d> getOpponentPosesDynamic(DriverStation.Alliance alliance, Time pollRate) {
        // If elapsed time is less than pollRate return our cached list.
        final var isBlue = alliance == DriverStation.Alliance.Blue;
        final var poseList = isBlue ? blueOpponentPoses : redOpponentPoses;
        if (System.currentTimeMillis() - (isBlue ? lastBluePosePoll : lastRedPosePoll) < pollRate.in(Milliseconds)) {
            return poseList;
        }
        // New list to filter.
        final var filteredOpponents = new ArrayList<>(opponents);
        // If elapsed time exceeds our pollRate, refresh the list.
        poseList.clear();
        filteredOpponents.stream()
                // Remove all opponents not on a given alliance.
                .filter(opponent -> opponent.config.alliance != alliance)
                // Collect all their poses.
                .forEach(opponent -> poseList.add(opponent.getOpponentPose()));
        // Update our refresh timestamp.
        if (isBlue) {
            lastBluePosePoll = System.currentTimeMillis();
        } else {
            lastRedPosePoll = System.currentTimeMillis();
        }
        return poseList;
    }

    /**
     * Returns all registered opponent poses.
     *
     * @return a list of all opponents poses on the given alliance.
     */
    public List<Pose2d> getOpponentPosesDynamic(Time pollRate)
    {
        // If elapsed time is less than pollRate return our cached list.
        if (System.currentTimeMillis() - lastPosePoll < pollRate.in(Milliseconds)) {
            return opponentPoses;
        }
        // If elapsed time exceeds our pollRate, refresh the list.
        opponentPoses.clear();
        // Call the other dynamic methods so we don't update if not needed.
        opponentPoses.addAll(getOpponentPosesDynamic(DriverStation.Alliance.Blue, pollRate));
        opponentPoses.addAll(getOpponentPosesDynamic(DriverStation.Alliance.Red, pollRate));
        // Update our refresh timestamp.
        lastPosePoll = System.currentTimeMillis();
        return opponentPoses;
    }

    /**
     * Saves a list globally in {@link OpponentManager} and returns that list only updating it if the last update
     * exceeds the given time.
     *
     * @param pollRate how long to wait before updating.
     * @return a list of obstacles usable by {@link MapleADStar}.
     */
    protected List<Pair<Translation2d, Translation2d>> getObstaclesDynamic(DriverStation.Alliance alliance, Time pollRate) {
        // If elapsed time is less than pollRate return our cached list.
        final var isBlue = alliance == DriverStation.Alliance.Blue;
        final var obstacleList = isBlue ? blueOpponentObstacles : redOpponentObstacles;
        if (System.currentTimeMillis() - (isBlue ? lastBlueObstaclePoll : lastRedObstaclePoll) < pollRate.in(Milliseconds)) {
            return obstacleList;
        }
        // New list to filter.
        final var targets = isBlue
                ? getOpponentTargetsDynamic(DriverStation.Alliance.Blue, pollRate)
                : getOpponentTargetsDynamic(DriverStation.Alliance.Red, pollRate);
        final var poses = isBlue
                ? getOpponentPosesDynamic(DriverStation.Alliance.Blue, pollRate)
                : getOpponentPosesDynamic(DriverStation.Alliance.Red, pollRate);
        obstacleList.clear();
        // Format poses into obstacles and add them.
        targets.forEach(target -> {
            obstacleList.add(poseToObstacle(Objects.requireNonNullElse(target.getSecond(), Pose2d.kZero)));
        });
        poses.forEach(pose -> {
            obstacleList.add(poseToObstacle(pose));
        });
        // Update our refresh timestamp.
        if (isBlue) {
            lastBlueObstaclePoll = System.currentTimeMillis();
        } else {
            lastRedObstaclePoll = System.currentTimeMillis();
        }
        return obstacleList;
    }

    /**
     * Saves a list globally in {@link OpponentManager} and returns that list only updating it if the last update
     * exceeds the given time.
     *
     * @param pollRate how long to wait before updating.
     * @return a list of obstacles usable by {@link MapleADStar}.
     */
    protected List<Pair<Translation2d, Translation2d>> getObstaclesDynamic(Time pollRate) {
        // If elapsed time is less than pollRate return our cached list.
        if (System.currentTimeMillis() - lastObstaclePoll < pollRate.in(Milliseconds)) {
            return opponentObstacles;
        }
        // If elapsed time exceeds our pollRate, refresh the list.
        opponentObstacles.clear();
        // Call the other dynamic methods so we don't update if not needed.
        opponentObstacles.addAll(getObstaclesDynamic(DriverStation.Alliance.Blue, pollRate));
        opponentObstacles.addAll(getObstaclesDynamic(DriverStation.Alliance.Red, pollRate));
        // Update our refresh timestamp.
        lastObstaclePoll = System.currentTimeMillis();
        return opponentObstacles;
    }

    /**
     * Formats a pose2d as a bounding box for obstacles using the boundingBoxBuffer.
     *
     * @param pose the pose to format.
     * @return an obstacle usable by pathplanner.
     */
    protected Pair<Translation2d, Translation2d> poseToObstacle(Pose2d pose) {
        return Pair.of(
                pose.getTranslation().plus(boundingBoxTranslation),
                pose.getTranslation().minus(boundingBoxTranslation)
        );
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
