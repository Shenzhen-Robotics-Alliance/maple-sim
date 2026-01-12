package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2026Rebuilt extends SimulatedArena {

    protected boolean shouldClock = true;

    protected double clock = 0;
    protected boolean blueIsOnClock = Math.random() < 0.5;

    protected DoublePublisher phaseClockPublisher =
            genericInfoTable.getDoubleTopic("Time left in current phase").publish();

    protected BooleanPublisher redActivePublisher =
            redTable.getBooleanTopic("Red is active").publish();
    protected BooleanPublisher blueActivePublisher =
            blueTable.getBooleanTopic("Blue is active").publish();

    protected RebuiltHub blueHub;
    protected RebuiltHub redHub;

    protected RebuiltOutpost blueOutpost;
    protected RebuiltOutpost redOutpost;

    protected boolean isInEfficiencyMode = true;

    protected static Translation2d centerPieceBottomRightCorner = new Translation2d(7.35737, 1.724406);
    protected static Translation2d redDepotBottomRightCorner = new Translation2d(0.02, 5.53);
    protected static Translation2d blueDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);

    /** the obstacles on the 2024 competition field */
    public static final class RebuiltFieldObstaclesMap extends FieldMap {
        private static final double FIELD_WIDTH = 16.54;

        public RebuiltFieldObstaclesMap(boolean AddRampCollider) {
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, 8.052));

            // red wall
            super.addBorderLine(new Translation2d(16.540988, 0), new Translation2d(16.540988, 8.052));

            // upper walls
            super.addBorderLine(new Translation2d(16.540988, 8.052), new Translation2d(0, 8.052));

            // lower walls
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(16.540988, 0));

            // Trench Walls (47 inch height, 12 inch width)
            double trenchWallDistX =
                    Inches.of(120.0).in(Meters) + Inches.of(47.0 / 2).in(Meters);

            double trenchWallDistY = Inches.of(73.0).in(Meters)
                    + Inches.of(47.0 / 2).in(Meters)
                    + Inches.of(6).in(Meters);

            super.addRectangularObstacle(
                    Inches.of(53).in(Meters),
                    Inches.of(12).in(Meters),
                    new Pose2d(8.27 - trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));
            super.addRectangularObstacle(
                    Inches.of(53).in(Meters),
                    Inches.of(12).in(Meters),
                    new Pose2d(8.27 + trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));
            super.addRectangularObstacle(
                    Inches.of(53).in(Meters),
                    Inches.of(12).in(Meters),
                    new Pose2d(8.27 - trenchWallDistX, 4.035 + trenchWallDistY, Rotation2d.kZero));
            super.addRectangularObstacle(
                    Inches.of(53).in(Meters),
                    Inches.of(12).in(Meters),
                    new Pose2d(8.27 - trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));

            // poles of the tower
            super.addRectangularObstacle(
                    Inches.of(2).in(Meters),
                    Inches.of(47).in(Meters),
                    new Pose2d(new Translation2d(Inches.of(42), Inches.of(159)), new Rotation2d()));

            super.addRectangularObstacle(
                    Inches.of(2).in(Meters),
                    Inches.of(47).in(Meters),
                    new Pose2d(new Translation2d(Inches.of(651 - 42), Inches.of(170)), new Rotation2d()));

            // Colliders to describe the hub plus ramps
            if (AddRampCollider) {
                super.addRectangularObstacle(
                        Inches.of(47).in(Meters),
                        Inches.of(217).in(Meters),
                        new Pose2d(RebuiltHub.blueHubPose.toTranslation2d(), new Rotation2d()));

                super.addRectangularObstacle(
                        Inches.of(47).in(Meters),
                        Inches.of(217).in(Meters),
                        new Pose2d(RebuiltHub.redHubPose.toTranslation2d(), new Rotation2d()));
            }

            // Colliders to describe just the hub
            else {
                super.addRectangularObstacle(
                        Inches.of(47).in(Meters),
                        Inches.of(47).in(Meters),
                        new Pose2d(RebuiltHub.blueHubPose.toTranslation2d(), new Rotation2d()));

                super.addRectangularObstacle(
                        Inches.of(47).in(Meters),
                        Inches.of(47).in(Meters),
                        new Pose2d(RebuiltHub.redHubPose.toTranslation2d(), new Rotation2d()));
            }
        }
    }
    /**
     *
     *
     * <h2>Creates an Arena for the 2026 FRC game rebuilt </h2>
     *
     * <p>This will create an Arena with the ramp areas marked as inaccessible. If you would like to change that use
     * {@link #Arena2026Rebuilt(boolean)}. Additionally due to performance issues the arena will not spawn all fuel by
     * default. If you would like to change this use {@link #setEfficiencyMode(boolean)}
     */
    public Arena2026Rebuilt() {
        this(true);
    }

    /**
     *
     *
     * <h2>Creates an Arena for the 2026 FRC game rebuilt </h2>
     *
     * <p>Due to the nature of maple sim they can not be fully simulated and so ether must be non existent or treated as
     * full colliders. This behavior can be changed with the AddRampCollider variable. Additionally due to performance
     * issues the arena will not spawn all fuel by default. If you would like to change this use
     * {@link #setEfficiencyMode(boolean)}
     *
     * @param AddRampCollider Whether or not the ramps should be added as colliders.
     */
    public Arena2026Rebuilt(boolean AddRampCollider) {
        super(new RebuiltFieldObstaclesMap(AddRampCollider));

        Settings settings = physicsWorld.getSettings();

        // settings.setVelocityConstraintSolverIterations(3);
        // settings.setPositionConstraintSolverIterations(2);
        settings.setMinimumAtRestTime(0.02);

        physicsWorld.setSettings(settings);

        blueHub = new RebuiltHub(this, true);
        super.addCustomSimulation(blueHub);

        redHub = new RebuiltHub(this, false);
        super.addCustomSimulation(redHub);

        blueOutpost = new RebuiltOutpost(this, true);
        super.addCustomSimulation(blueOutpost);

        redOutpost = new RebuiltOutpost(this, false);
        super.addCustomSimulation(redOutpost);
    }

    /**
     *
     *
     * <h2>Generates a random number within a range centered on 0 with a variance set by the parameter. </h2>
     *
     * @param variance the length of range used to generate the random number.
     * @return A random number in range.
     */
    public static double randomInRange(double variance) {
        return (Math.random() - 0.5) * variance;
    }

    /**
     *
     *
     * <h2>Adds a game piece too the arena with a certain random variance.</h2>
     *
     * This method is useful for certain spawners like the return cutes on the hub to prevent the game pieces from being
     * returned to the exact same position every time.
     *
     * @param info the info of the game piece
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisSpeedsFieldRelative the field-relative velocity of the robot chassis when launching the game piece,
     *     influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingSpeed the speed at which the game piece is launch
     * @param shooterAngle the pitch angle of the shooter when launching
     * @param xVariance The max amount of variance that should be added too the x coordinate of the game piece.
     * @param yVariance The max amount of variance that should be added too the y coordinate of the game piece.
     * @param yawVariance The max amount of variance that should be added too the yaw of the game piece.
     * @param speedVariance The max amount of variance that should be added too the speed of the game piece.
     * @param pitchVariance The max amount of variance that should be added too the pitch of the game piece.
     */
    public void addPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
            Angle pitch,
            double xVariance,
            double yVariance,
            double yawVariance,
            double speedVariance,
            double pitchVariance) {
        addGamePieceProjectile(new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
                new Translation2d(),
                new ChassisSpeeds(),
                yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
                height,
                speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
                Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance))));
    }

    @Override
    public void placeGamePiecesOnField() {
        blueOutpost.reset();
        redOutpost.reset();

        for (int x = 0; x < 12; x += 1) {
            for (int y = 0; y < 30; y += isInEfficiencyMode ? 3 : 1) {
                addGamePiece(new RebuiltFuelOnField(centerPieceBottomRightCorner.plus(
                        new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
            }
        }

        boolean isOnBlue = !DriverStation.getAlliance().isEmpty()
                && DriverStation.getAlliance().get() == Alliance.Blue;

        if (isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new RebuiltFuelOnField(blueDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
        }

        if (!isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new RebuiltFuelOnField(redDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        blueOutpost.draw(poses);
        redOutpost.draw(poses);

        return poses;
    }

    @Override
    public void simulationSubTick(int tickNum) {

        clock -= getSimulationDt().in(Units.Seconds);

        if (clock <= 0) {
            clock = 25;
            blueIsOnClock = !blueIsOnClock;
        }

        super.simulationSubTick(tickNum);

        blueActivePublisher.set(isActive(true));
        redActivePublisher.set(isActive(false));

        phaseClockPublisher.set((shouldClock || DriverStation.isAutonomous() ? clock : null));
    }

    /**
     *
     *
     * <h2>Returns wether the specified team currently has an active HUB </h2>
     *
     * This function returns true during autonomous or when shouldClock (set by {@link #setShouldRunClock(boolean)}) is
     * false.
     *
     * @param isBlue Wether to check the blue or red alliance.
     * @return Wether the specified alliance's HUB is currently active
     */
    public boolean isActive(boolean isBlue) {
        if (isBlue) {
            return blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        } else {
            return !blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        }
    }

    /**
     *
     *
     * <h2>Used to determine wether the arena should time which goal is active. </h2>
     *
     * When this is set too false both goals will always be set to active. Ths can be useful for testing or too simulate
     * endgame.
     *
     * @param shouldRunClock
     */
    public void setShouldRunClock(boolean shouldRunClock) {
        shouldClock = shouldRunClock;
    }

    /**
     *
     *
     * <h2>Dumps game pieces from the specified outpost.</h2>
     *
     * This function will dump up to 24 game pieces, dependent on how many game pieces are currently stored in the
     * outpost. For more manual control of the game pieces in the outpost use {@link #outpostThrow(boolean, Rotation2d,
     * Angle, LinearVelocity)}. To have a human player attempt to throw a game piece into the hub use
     * {@link #outpostThrowForGoal(boolean)}.
     *
     * @param isBlue wether to dump the blue or red outpost
     */
    public void outpostDump(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).dump();
    }

    /**
     *
     *
     * <h2>Attempts too throw a game piece at the specified goal.</h2>
     *
     * <p>This method comes with variance built in (to simulate human inconsistency) and will therefore only hit about
     * half the time. Additionally if the hub does not have game pieces stored this method will not do anything. If you
     * would like to manually control how the human player throws game pieces use {@link #outpostThrow(boolean,
     * Rotation2d, Angle, LinearVelocity)}
     *
     * @param isBlue whether too throw for the blue or red HUB.
     */
    public void outpostThrowForGoal(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).throwForGoal();
    }

    /**
     *
     *
     * <h2>Throws a game piece from the outpost at the specified angle and speed.</h2>
     *
     * <p>This method comes with variance built in (to simulate human inconsistency). Additionally if the hub does not
     * have game pieces stored this method will not do anything. If you would like to have the human player throw at the
     * hub use {@link #outpostThrowForGoal(boolean)}
     *
     * @param isBlue Wether too throw from the blue or red OUTPOST.
     * @param throwYaw The yaw at which too throw the ball.
     * @param throwPitch The pitch at which too throw the ball.
     * @param speed The speed at which too throw the ball.
     */
    public void outpostThrow(boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed) {
        (isBlue ? blueOutpost : redOutpost).throwFuel(throwYaw, throwPitch, speed);
    }

    /**
     *
     *
     * <h2>Determines wether the arena is in efficiency mode </h2>
     *
     * <h3>For changes too take effect call {@link #resetFieldForAuto()}. </h3>
     *
     * <p>Efficiency mode reduces the amount of game pieces on the field too increase performance. MapleSim was not
     * designed with 400 game pieces in mind and so can struggle with the large number of game pieces present in
     * rebuilt.
     *
     * @param efficiencyMode Wether efficiency mode should be on or off.
     */
    public void setEfficiencyMode(boolean efficiencyMode) {
        isInEfficiencyMode = efficiencyMode;
    }

    /**
     *
     *
     * <h2>Returns wether or not the arena is in Efficiency mode </h2>
     *
     * For more information see {@link #setEfficiencyMode(boolean)}
     *
     * @return Wether or not the Arena is in efficiency mode.
     */
    public boolean getEfficiencyMode() {
        return isInEfficiencyMode;
    }
}
