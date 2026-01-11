package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
import org.ironmaple.simulation.SimulatedArena;

public class Arena2026Rebuilt extends SimulatedArena {

    protected boolean shouldClock = true;

    protected double clock = 0;
    protected boolean blueIsOnClock = true;

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

    protected static Translation2d centerPieceBottomRightCorner = new Translation2d(8.270494, 1.724406);

    /** the obstacles on the 2024 competition field */
    public static final class RebuiltFieldObstaclesMap extends FieldMap {
        private static final double FIELD_WIDTH = 16.54;

        public RebuiltFieldObstaclesMap() {
            super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

            // red wall
            super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

            // upper walls
            super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(11, 8.052));
            super.addBorderLine(new Translation2d(12, 8.052), new Translation2d(17.548 - 1.672, 8.052));

            // lower walls
            super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(5.8, 0));
            super.addBorderLine(new Translation2d(6.3, 0), new Translation2d(17.548 - 1.672, 0));
        }
    }

    public Arena2026Rebuilt() {
        super(new RebuiltFieldObstaclesMap());

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
        return (Math.random() - 0.5) / variance;
    }

    public void addPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
            Angle pitch,
            double positionVariance,
            double yawVariance,
            double speedVariance,
            double pitchVariance) {
        addGamePieceProjectile(new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(randomInRange(positionVariance), randomInRange(positionVariance))),
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

        for (int x = 0; x < 12; x++) {
            for (int y = 0; y < 30; y++) {
                addGamePiece(new RebuiltFuelOnField(centerPieceBottomRightCorner.plus(
                        new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }

    @Override
    public void simulationSubTick(int tickNum) {

        clock -= getSimulationDt().in(Units.Seconds);

        if (clock <= 0) {
            clock = 25;
            blueIsOnClock = !blueIsOnClock;
        }

        super.simulationSubTick(tickNum);

        blueActivePublisher.set(shouldClock || blueIsOnClock);
        redActivePublisher.set(shouldClock || !blueIsOnClock);

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
}
