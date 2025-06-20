package org.ironmaple.simulation.seasonspecific.crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2024Crescendo extends SimulatedArena {

    protected double blueAmpClock = 0;
    protected int blueAmpCount = 0;

    protected double redAmpClock = 0;
    protected int redAmpCount = 0;

    protected final CrescendoSpeaker redSpeaker;
    protected final CrescendoSpeaker blueSpeaker;

    protected final CrescendoAmp redAmp;
    protected final CrescendoAmp blueAmp;

    /** the obstacles on the 2024 competition field */
    public static final class CrescendoFieldObstaclesMap extends FieldMap {
        private static final double FIELD_WIDTH = 16.54;

        public CrescendoFieldObstaclesMap() {
            super();

            // left wall
            super.addBorderLine(new Translation2d(0, 1), new Translation2d(0, 4.51));
            super.addBorderLine(new Translation2d(0, 4.51), new Translation2d(0.9, 5));

            super.addBorderLine(new Translation2d(0.9, 5), new Translation2d(0.9, 6.05));

            super.addBorderLine(new Translation2d(0.9, 6.05), new Translation2d(0, 6.5));
            super.addBorderLine(new Translation2d(0, 6.5), new Translation2d(0, 8.2));

            // upper wall
            super.addBorderLine(new Translation2d(0, 8.12), new Translation2d(FIELD_WIDTH, 8.12));

            // righter wall
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 1), new Translation2d(FIELD_WIDTH, 4.51));
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 4.51), new Translation2d(FIELD_WIDTH - 0.9, 5));
            super.addBorderLine(new Translation2d(FIELD_WIDTH - 0.9, 5), new Translation2d(FIELD_WIDTH - 0.9, 6.05));
            super.addBorderLine(new Translation2d(FIELD_WIDTH - 0.9, 6.05), new Translation2d(FIELD_WIDTH, 6.5));
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 6.5), new Translation2d(FIELD_WIDTH, 8.2));

            // lower wall
            super.addBorderLine(new Translation2d(1.92, 0), new Translation2d(FIELD_WIDTH - 1.92, 0));

            // red source wall
            super.addBorderLine(new Translation2d(1.92, 0), new Translation2d(0, 1));

            // blue source wall
            super.addBorderLine(new Translation2d(FIELD_WIDTH - 1.92, 0), new Translation2d(FIELD_WIDTH, 1));

            // blue state
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(3.4, 4.1, new Rotation2d()));
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(5.62, 4.1 - 1.28, Rotation2d.fromDegrees(30)));
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(5.62, 4.1 + 1.28, Rotation2d.fromDegrees(60)));

            // red stage
            super.addRectangularObstacle(0.35, 0.35, new Pose2d(FIELD_WIDTH - 3.4, 4.1, new Rotation2d()));
            super.addRectangularObstacle(
                    0.35, 0.35, new Pose2d(FIELD_WIDTH - 5.62, 4.1 - 1.28, Rotation2d.fromDegrees(60)));
            super.addRectangularObstacle(
                    0.35, 0.35, new Pose2d(FIELD_WIDTH - 5.62, 4.1 + 1.28, Rotation2d.fromDegrees(30)));
        }
    }

    private static final Translation2d[] NOTE_INITIAL_POSITIONS = new Translation2d[] {
        new Translation2d(2.9, 4.1),
        new Translation2d(2.9, 5.55),
        new Translation2d(2.9, 7),
        new Translation2d(8.27, 0.75),
        new Translation2d(8.27, 2.43),
        new Translation2d(8.27, 4.1),
        new Translation2d(8.27, 5.78),
        new Translation2d(8.27, 7.46),
        new Translation2d(13.64, 4.1),
        new Translation2d(13.64, 5.55),
        new Translation2d(13.64, 7),
    };

    public Arena2024Crescendo() {
        super(new CrescendoFieldObstaclesMap());

        redSpeaker = new CrescendoSpeaker(this, false);
        super.addCustomSimulation(redSpeaker);

        blueSpeaker = new CrescendoSpeaker(this, true);
        super.addCustomSimulation(blueSpeaker);

        blueAmp = new CrescendoAmp(this, true);
        super.addCustomSimulation(blueAmp);

        redAmp = new CrescendoAmp(this, false);
        super.addCustomSimulation(redAmp);
    }

    @Override
    public void placeGamePiecesOnField() {
        for (Translation2d notePosition : NOTE_INITIAL_POSITIONS)
            super.addGamePiece(new CrescendoNoteOnField(notePosition));

        super.addCustomSimulation(new CrescendoHumanPlayerSimulation(this));
    }

    @Override
    public void simulationSubTick(int tickNum) {
        redAmpClock -= getSimulationDt().in(Units.Seconds);
        blueAmpClock -= getSimulationDt().in(Units.Seconds);
        super.simulationSubTick(tickNum);

        SmartDashboard.putBoolean("Red is amped", isAmped(false));
        SmartDashboard.putBoolean("Blue is amped", isAmped(true));
        SmartDashboard.putNumber("blue Amp Charge", blueAmpCount);
        SmartDashboard.putNumber("red Amp Charge", redAmpCount);
        SmartDashboard.putNumber("blue Amp Clock", blueAmpClock);
        SmartDashboard.putNumber("red Amp Clock", redAmpClock);
    }

    public boolean isAmped(boolean isBlue) {
        if (isBlue) {
            return blueAmpClock > 0 || DriverStation.isAutonomous();
        } else {
            return redAmpClock > 0 || DriverStation.isAutonomous();
        }
    }

    public void addAmpCharge(boolean isBlue) {
        if (isBlue) {
            blueAmpCount = Math.min(blueAmpCount + 1, 2);
        } else {
            redAmpCount = Math.min(redAmpCount + 1, 2);
        }
    }

    public boolean activateAmp(boolean isBlue) {
        if (isBlue && blueAmpCount == 2) {
            blueAmpCount = 0;
            blueAmpClock = 5;
            System.out.println("blue amped it up");
            return true;
        }
        if (!isBlue && redAmpCount == 2) {
            redAmpCount = 0;
            redAmpClock = 5;
            System.out.println("red amped it up");
            return true;
        }
        System.out.println(
                isBlue
                        ? "Blue "
                        : "Red " + "Only has " + (isBlue ? blueAmpCount : redAmpCount) + "of needed 2 to use amp");
        return false;
    }
}
