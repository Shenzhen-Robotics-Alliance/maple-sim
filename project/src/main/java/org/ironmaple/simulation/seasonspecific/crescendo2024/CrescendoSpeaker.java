package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.*;
import org.ironmaple.simulation.gamepieces.GamePieceInterface;
import org.ironmaple.simulation.goal;

/**
 *
 *
 * <h2>Simulates the two <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates the two <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored. It
 * includes all 12 {@link ReefscapeReefBranchesTower} instances on the field (both blue and red).
 */
public class CrescendoSpeaker extends goal {

    protected static final Translation3d redSpeakerPose = new Translation3d(16.52, 5.5825, 2.1);
    protected static final Translation3d blueSpeakerPose = new Translation3d(0, 5.5825, 2.1);
    StructPublisher<Pose3d> posePublisher;
    protected final Arena2024Crescendo crescendoArena;

    public CrescendoSpeaker(Arena2024Crescendo arena, boolean isBlue) {
        super(
                arena,
                Centimeters.of(90),
                Centimeters.of(116.5),
                Centimeters.of(100),
                "Note",
                isBlue ? blueSpeakerPose : redSpeakerPose,
                isBlue);
        // setNeededVelAngle(new Rotation3d(0, Math.PI / 2, 0), Degrees.of(90));
        crescendoArena = arena;
        StructPublisher<Pose3d> speakerPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueSpeaker" : "RedSpeaker", Pose3d.struct)
                .publish();
        speakerPosePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected boolean checkVel(GamePieceInterface gamePiece) {
        return gamePiece.getVelocity3dMPS().getZ() > 0;
    }

    @Override
    protected void addPoints() {
        arena.addToScore(isBlue, crescendoArena.isAmped(isBlue) ? 5 : 2);
        
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        return;
    }

    // 6.165
    // 0.45, 5
}
