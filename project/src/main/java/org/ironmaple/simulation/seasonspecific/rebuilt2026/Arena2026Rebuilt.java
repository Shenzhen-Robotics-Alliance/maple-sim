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

    public Arena2026Rebuilt() {
        this(true);
    }

    public Arena2026Rebuilt(boolean AddRampCollider) {
        super(new RebuiltFieldObstaclesMap(AddRampCollider));

        Settings settings = physicsWorld.getSettings();

        // settings.setVelocityConstraintSolverIterations(3);
        // settings.setPositionConstraintSolverIterations(2);

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

    public static double randomInRange(double variance) {
        return (Math.random() - 0.5) * variance;
    }

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
     * <h2>Returns wether the specified team currently has an amped speaker</h2>
     *
     * This function returns true during autonomous since the autonomous mode behaves exactly like amplified game play.
     *
     * @param isBlue Wether to check the blue or red alliance.
     * @return Wether the specified alliance is currently amplified.
     */
    public boolean isActive(boolean isBlue) {
        if (isBlue) {
            return blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        } else {
            return !blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        }
    }

    public void setShouldRunClock(boolean shouldRunClock) {
        shouldClock = shouldRunClock;
    }

    public void outpostDump(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).dump();
    }

    public void outpostThrowForGoal(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).throwForGoal();
    }

    public void outpostThrow(boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed) {
        (isBlue ? blueOutpost : redOutpost).throwAtPoint(throwYaw, throwPitch, speed);
    }

    public void setEfficiencyMode(boolean efficiencyMode) {
        isInEfficiencyMode = efficiencyMode;
    }

    public boolean getEfficiencyMode() {
        return isInEfficiencyMode;
    }
}
