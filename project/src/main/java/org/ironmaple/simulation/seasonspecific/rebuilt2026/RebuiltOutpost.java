package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import java.util.*;
import org.ironmaple.simulation.Goal;

/**
 *
 *
 * <h2>Simulates a <strong>PROCESSOR</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>PROCESSOR</strong>s on the field where <strong>ALGAE</strong>s can be scored. It
 * will automatically launch the algae scored into the apposing barge and always hit.
 */
public class RebuiltOutpost extends Goal {

    protected static final Translation3d blueOutpostPose = new Translation3d(8.0518, 15.621, 0);
    protected static final Translation3d redOutpostPose = new Translation3d(0.665988, -0.254, 0);
    protected static final Translation3d blueLaunchPose = new Translation3d(7.8, 16, 0);
    protected static final Translation3d redLaunchPose = new Translation3d(-0.2, 0.254, 0);
    protected static final Translation3d blueDumpPose = new Translation3d(7.8, 15.621, 0);
    protected static final Translation3d redDumpPose = new Translation3d(0.665988, 0.254, 0);

    StructPublisher<Pose3d> posePublisher;

    protected int fuelCount = 24;

    protected Arena2026Rebuilt arena;

    /**
     *
     *
     * <h2>Creates an processor of the specified color.</h2>
     *
     * @param arena The host arena of this processor.
     * @param isBlue Wether this is the blue processor or the red one.
     */
    public RebuiltOutpost(Arena2026Rebuilt arena, boolean isBlue) {
        super(
                arena,
                Inches.of(21),
                Centimeters.of(26),
                Centimeters.of(10),
                "Fuel",
                isBlue ? blueLaunchPose : redLaunchPose,
                isBlue);

        this.arena = arena;

        StructPublisher<Pose3d> heldAlgaePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueOutpost" : "RedOutpost", Pose3d.struct)
                .publish();
        heldAlgaePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInOutpost", 1);
        this.fuelCount++;
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        super.simulationSubTick(subTickNum);
        arena.addValueToMatchBreakdown(isBlue, "CurrentFuelInOutpost", fuelCount);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        return;
    }

    public void reset() {
        fuelCount = 24;
    }

    public void throwForGoal() {
        throwAtPoint(null, pieceAngleTolerance);
    }

    public void dump() {
        for (int i = 0; i < 24 && fuelCount > 0; i++) {
            this.arena.addPieceWithVariance(
                    blueDumpPose.toTranslation2d(),
                    isBlue ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0),
                    Meters.of(1.7),
                    MetersPerSecond.of(7),
                    Degrees.of(0),
                    0.2,
                    5,
                    0.2,
                    5.0);
        }
    }

    public void throwAtPoint(Rotation2d yaw, Angle pitch) {

        arena.addPieceWithVariance(
                isBlue ? blueLaunchPose.toTranslation2d() : redLaunchPose.toTranslation2d(),
                yaw,
                Meters.of(1.7),
                MetersPerSecond.of(7),
                pitch,
                0,
                2,
                0,
                0);
    }
}
