package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.*;
import org.ironmaple.simulation.goal;

/**
 *
 *
 * <h2>Simulates a <strong>PROCESSOR</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>PROCESSOR</strong>s on the field where <strong>ALGAE</strong>s can be scored. 
 * It will automatically launch the algae scored into the apposing barge and always hit.
 */
public class ReefscapeProcessorSimulation extends goal {

    protected static final Translation3d blueProcessorPose = new Translation3d(6.34, -0.5, 0);
    protected static final Translation3d redProcessorPose = new Translation3d(11.5, 8.5, 0);
    protected static final Translation3d blueProcessorLaunchPose = new Translation3d(6.34, 0, 0);
    protected static final Translation3d redProcessorLaunchPose = new Translation3d(11.5, 8, 0);

    StructPublisher<Pose3d> posePublisher;


    /**
     * <h2>Creates an processor of the specified color.</h2>
     * @param arena The host arena of this processor.
     * @param isBlue Wether this is the blue processor or the red one.
     */
    public ReefscapeProcessorSimulation(Arena2025Reefscape arena, boolean isBlue) {
        super(
                arena,
                Centimeters.of(100),
                Centimeters.of(100),
                Centimeters.of(100),
                "Algae",
                isBlue ? blueProcessorPose : redProcessorPose,
                isBlue);

        StructPublisher<Pose3d> heldAlgaePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueProcessor" : "RedProcessor", Pose3d.struct)
                .publish();
        heldAlgaePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected void addPoints() {
        if (isBlue) {
            arena.addToScore(isBlue, 6);
            this.arena.addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                    blueProcessorLaunchPose.toTranslation2d(),
                    new Translation2d(),
                    new ChassisSpeeds(),
                    Rotation2d.fromDegrees(45),
                    Meters.of(1.7),
                    MetersPerSecond.of(7),
                    Degrees.of(45)));
        } else {
            arena.addToScore(isBlue, 6);
            this.arena.addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                    redProcessorLaunchPose.toTranslation2d(),
                    new Translation2d(),
                    new ChassisSpeeds(),
                    Rotation2d.fromDegrees(-135),
                    Meters.of(1.7),
                    MetersPerSecond.of(7),
                    Degrees.of(45)));
        }
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        return;
    }

    
}
