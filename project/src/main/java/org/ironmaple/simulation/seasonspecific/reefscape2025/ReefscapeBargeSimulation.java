package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;
import org.ironmaple.simulation.Goal;

/**
 *
 *
 * <h2>Simulates a <strong>BARGE</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>BARGE</strong>s on the field where <strong>ALGAE</strong> can be scored.
 */
public class ReefscapeBargeSimulation extends Goal {

    protected static final Translation3d redBargePose = new Translation3d(8.805, 2.1, 1.57);
    protected static final Translation3d blueBargePose = new Translation3d(8.805, 6.18, 1.57);
    StructPublisher<Pose3d> posePublisher;

    /**
     *
     *
     * <h2>Creates an barge of the specified color.</h2>
     *
     * @param arena The host arena of this barge.
     * @param isBlue Wether this is the blue barge or the red one.
     */
    public ReefscapeBargeSimulation(Arena2025Reefscape arena, boolean isBlue) {
        super(
                arena,
                Centimeters.of(112),
                Centimeters.of(343),
                Centimeters.of(100),
                "Algae",
                isBlue ? blueBargePose : redBargePose,
                isBlue,
                false);

        withCustomVelocityValidator(gamePiece -> gamePiece.getVelocity3dMPS().getZ() < 0);

        StructPublisher<Pose3d> heldAlgaePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("/SmartDashboard/MapleSim/" + (isBlue ? "BlueBarge" : "RedBarge"), Pose3d.struct)
                .publish();
        heldAlgaePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    public void draw(List<Pose3d> algaePosesToDisplay) {
        for (int i = 0; i < gamePieceCount; i++) {
            algaePosesToDisplay.add(new Pose3d(
                            position.getX(),
                            position.getY() - Centimeters.of(343).in(Units.Meters) / 2,
                            position.getZ() + Centimeters.of(100).in(Units.Meters) / 2,
                            new Rotation3d())
                    .plus(new Transform3d(0, i * 0.35, 0, new Rotation3d())));
        }
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "AlgaeInNet", 1);
        arena.addValueToMatchBreakdown(isBlue, "Auto/AlgaeScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);

        arena.addToScore(isBlue, 4);
    }
}
