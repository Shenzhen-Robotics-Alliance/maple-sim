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
 * <h2>Simulates the two <strong>REEF</strong>s on the field.</h2>
 *
 * <p>This class simulates the two <strong>REEF</strong>s on the field where <strong>CORAL</strong>s can be scored. It
 * includes all 12 {@link ReefscapeReefBranchesTower} instances on the field (both blue and red).
 */
public class ReefscapeProcessorSimulation extends goal {

    protected static final Translation3d blueProcessorPose = new Translation3d(6.34, -0.5, 0);
    protected static final Translation3d redProcessorPose = new Translation3d(11.5, 8.5, 0);
    protected static final Translation3d blueProcessorLaunchPose = new Translation3d(6.34, 0, 0);
    protected static final Translation3d redProcessorLaunchPose = new Translation3d(11.5, 8, 0);

    StructPublisher<Pose3d> posePublisher;

    public ReefscapeProcessorSimulation(Arena2025Reefscape arena, boolean isBlue) {
        super(
                arena,
                Centimeters.of(100),
                Centimeters.of(100),
                Centimeters.of(100),
                "Algae",
                isBlue ? blueProcessorPose : redProcessorPose,
                isBlue);
        setNeededVelAngle(new Rotation3d(0, 90, 0), 90);

        StructPublisher<Pose3d> heldAlgaePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueProcessor" : "RedProcessor", Pose3d.struct)
                .publish();
        heldAlgaePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected void addPoints() {
        if (isBlue) {
            arena.addToBlueScore(6);
            this.arena.addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                    blueProcessorLaunchPose.toTranslation2d(),
                    new Translation2d(),
                    new ChassisSpeeds(),
                    Rotation2d.fromDegrees(45),
                    Meters.of(1.7),
                    MetersPerSecond.of(7),
                    Degrees.of(45)));
        } else {
            arena.addToRedScore(6);
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
