package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static org.ironmaple.utils.LegacyFieldMirroringUtils2024.toCurrentAllianceTranslation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class CrescendoHumanPlayerSimulation implements SimulatedArena.Simulatable {
    private static final Translation2d BLUE_SOURCE_POSITION = new Translation2d(15.6, 0.8);
    private double previousThrowTimeSeconds = 0;
    private final Arena2024Crescendo arena;

    public CrescendoHumanPlayerSimulation(Arena2024Crescendo arena) {
        this.arena = arena;
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (!DriverStation.isTeleopEnabled()) return;

        if (Timer.getFPGATimestamp() - previousThrowTimeSeconds < 1) return;

        final Translation2d sourcePosition = toCurrentAllianceTranslation(BLUE_SOURCE_POSITION);
        /* if there is any game-piece 0.5 meters within the human player station, we don't throw a new note */
        for (GamePieceOnFieldSimulation gamePiece : arena.gamePiecesOnField())
            if (gamePiece instanceof CrescendoNoteOnField
                    && gamePiece.getPoseOnField().getTranslation().getDistance(sourcePosition) < 1) return;

        /* otherwise, place a note */
        arena.addGamePiece(new CrescendoNoteOnField(sourcePosition));
        previousThrowTimeSeconds = Timer.getFPGATimestamp();
    }
}
