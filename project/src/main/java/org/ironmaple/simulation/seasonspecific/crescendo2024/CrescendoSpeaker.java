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
 * <h2>Simulates a <strong>SPEAKER</strong>s on the field.</h2>
 *
 * <p>This class simulates the <strong>SPEAKER</strong>s on the field where <strong>NOTES</strong>s can be scored. 
 * It may be amplified by using the {@link Arena2024Crescendo#activateAmp(boolean)} function on the host arena.
 */
public class CrescendoSpeaker extends goal {

    protected static final Translation3d redSpeakerPose = new Translation3d(16.52, 5.5825, 2.1);
    protected static final Translation3d blueSpeakerPose = new Translation3d(0, 5.5825, 2.1);
    StructPublisher<Pose3d> posePublisher;
    protected final Arena2024Crescendo crescendoArena;


    /**
     * <h2>Creates an Speaker of the specified color.</h2>
     * @param arena The host arena of this speaker.
     * @param isBlue Wether this is the blue speaker or the red one.
     */
    public CrescendoSpeaker(Arena2024Crescendo arena, boolean isBlue) {
        super(
                arena,
                Centimeters.of(90),
                Centimeters.of(116.5),
                Centimeters.of(100),
                "Note",
                isBlue ? blueSpeakerPose : redSpeakerPose,
                isBlue);

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

}
