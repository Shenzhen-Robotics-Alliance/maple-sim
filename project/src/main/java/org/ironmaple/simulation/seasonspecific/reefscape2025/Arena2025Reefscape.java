package org.ironmaple.simulation.seasonspecific.reefscape2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.FieldMirroringUtils;

/**
 *
 *
 * <h1>The playing field for the 2025 FRC Game: Reefscape</h1>
 *
 * <p>This class represents the playing field for the 2025 FRC game, Reefscape.
 *
 * <p>It extends {@link SimulatedArena} and includes specific details of the Reefscape game environment.
 */
public class Arena2025Reefscape extends SimulatedArena {
    public static final class ReefscapeFieldObstacleMap extends FieldMap {
        public ReefscapeFieldObstacleMap() {
            super();

            // blue wall
            super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

            // blue coral stations
            super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672, 0));
            super.addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052));

            // red wall
            super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

            // red coral stations
            super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548 - 1.672, 0));
            super.addBorderLine(new Translation2d(17.548, 6.782), new Translation2d(17.548 - 1.672, 8.052));

            // upper walls
            super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(11, 8.052));
            super.addBorderLine(new Translation2d(12, 8.052), new Translation2d(17.548 - 1.672, 8.052));

            // lower walls
            super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(5.8, 0));
            super.addBorderLine(new Translation2d(6.3, 0), new Translation2d(17.548 - 1.672, 0));

            // blue reef
            Translation2d[] reefVorticesBlue = new Translation2d[] {
                new Translation2d(3.658, 3.546),
                new Translation2d(3.658, 4.506),
                new Translation2d(4.489, 4.987),
                new Translation2d(5.3213, 4.506),
                new Translation2d(5.3213, 3.546),
                new Translation2d(4.489, 3.065)
            };
            for (int i = 0; i < 6; i++) super.addBorderLine(reefVorticesBlue[i], reefVorticesBlue[(i + 1) % 6]);

            // red reef
            Translation2d[] reefVorticesRed = Arrays.stream(reefVorticesBlue)
                    .map(pointAtBlue ->
                            new Translation2d(FieldMirroringUtils.FIELD_WIDTH - pointAtBlue.getX(), pointAtBlue.getY()))
                    .toArray(Translation2d[]::new);
            for (int i = 0; i < 6; i++) super.addBorderLine(reefVorticesRed[i], reefVorticesRed[(i + 1) % 6]);

            // the pillar in the middle of the field
            super.addRectangularObstacle(0.305, 0.305, new Pose2d(8.774, 4.026, new Rotation2d()));
        }
    }

    public final ReefscapeReefSimulation redReefSimulation;
    public final ReefscapeReefSimulation blueReefSimulation;
    public final ReefscapeBargeSimulation redBarge;
    public final ReefscapeBargeSimulation blueBarge;
    public final ReefscapeProcessorSimulation redProcessor;
    public final ReefscapeProcessorSimulation blueProcessor;

    public Arena2025Reefscape() {
        super(new ReefscapeFieldObstacleMap());

        redReefSimulation = new ReefscapeReefSimulation(this, false);
        super.addCustomSimulation(redReefSimulation);

        blueReefSimulation = new ReefscapeReefSimulation(this, true);
        super.addCustomSimulation(blueReefSimulation);

        blueBarge = new ReefscapeBargeSimulation(this, true);
        super.addCustomSimulation(blueBarge);

        redBarge = new ReefscapeBargeSimulation(this, false);
        super.addCustomSimulation(redBarge);

        blueProcessor = new ReefscapeProcessorSimulation(this, true);
        super.addCustomSimulation(blueProcessor);

        redProcessor = new ReefscapeProcessorSimulation(this, false);
        super.addCustomSimulation(redProcessor);

        setupValueForMatchBreakdown("AlgaeInProcessor");
        setupValueForMatchBreakdown("AlgaeInNet");
        setupValueForMatchBreakdown("CoralScoredInAuto");
        setupValueForMatchBreakdown("CoralScoredOnLevel 1");
        setupValueForMatchBreakdown("CoralScoredOnLevel 2");
        setupValueForMatchBreakdown("CoralScoredOnLevel 3");
        setupValueForMatchBreakdown("CoralScoredOnLevel 4");
        setupValueForMatchBreakdown("TotalCoralScored");
    }

    @Override
    public void placeGamePiecesOnField() {
        Translation2d[] bluePositions = new Translation2d[] {
            new Translation2d(1.219, 5.855), new Translation2d(1.219, 4.026), new Translation2d(1.219, 2.197),
        };
        for (Translation2d position : bluePositions) super.addGamePiece(new ReefscapeCoralAlgaeStack(position));

        Translation2d[] redPositions = Arrays.stream(bluePositions)
                .map(bluePosition ->
                        new Translation2d(FieldMirroringUtils.FIELD_WIDTH - bluePosition.getX(), bluePosition.getY()))
                .toArray(Translation2d[]::new);
        for (Translation2d position : redPositions) super.addGamePiece(new ReefscapeCoralAlgaeStack(position));
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        // add algae and coral stack
        if (type.equals("Algae")) {
            poses.addAll(ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
            redBarge.draw(poses);
            blueBarge.draw(poses);

        } else if (type.equals("Coral")) {
            poses.addAll(ReefscapeCoralAlgaeStack.getStackedCoralPoses());
            redReefSimulation.draw(poses);
            blueReefSimulation.draw(poses);
        }

        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        redReefSimulation.clearReef();
        blueReefSimulation.clearReef();
    }
}
