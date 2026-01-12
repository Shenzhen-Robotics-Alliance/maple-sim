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
import edu.wpi.first.units.measure.LinearVelocity;
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

    protected static final Translation3d redOutpostPose = new Translation3d(16.621, 7.403338, 0);
    protected static final Translation3d redLaunchPose = new Translation3d(16, 8.2, 0);
    protected static final Translation3d redDumpPose = new Translation3d(16.421, 7.2, 0);

    protected static final Translation3d blueOutpostPose = new Translation3d(0, 0.665988, 0);
    protected static final Translation3d blueDumpPose = new Translation3d(0.2, 0.665988, 0);
    protected static final Translation3d blueLaunchPose = new Translation3d(0.665988, -0.254, 0);

    protected static final Translation3d redRenderPose = new Translation3d(16.640988, 7.7, 0.844502);
    protected static final Translation3d blueRenderPose = new Translation3d(-0.12, 0.325, 0.844502);

    StructPublisher<Pose3d> posePublisher;

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
                Centimeters.of(3),
                Inches.of(21),
                Centimeters.of(10),
                "Fuel",
                isBlue ? blueOutpostPose : redOutpostPose,
                isBlue,
                true);

        this.arena = arena;
        gamePieceCount = 24;

        StructPublisher<Pose3d> OutpostPublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueOutpost" : "RedOutpost", Pose3d.struct)
                .publish();

        StructPublisher<Pose3d> OutpostThrowPublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueOutpostThrow" : "RedOutpostThrow", Pose3d.struct)
                .publish();
        StructPublisher<Pose3d> OutpostDumpPublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueOutpostDump" : "RedOutpostDump", Pose3d.struct)
                .publish();

        OutpostPublisher.set(new Pose3d(position, new Rotation3d()));
        OutpostDumpPublisher.set(new Pose3d(isBlue ? blueDumpPose : redDumpPose, new Rotation3d()));
        OutpostThrowPublisher.set(new Pose3d(isBlue ? blueLaunchPose : redLaunchPose, new Rotation3d()));
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInOutpost", 1);
        this.gamePieceCount++;
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        super.simulationSubTick(subTickNum);
        arena.replaceValueInMatchBreakDown(isBlue, "CurrentFuelInOutpost", gamePieceCount);
    }

    @Override
    public void draw(List<Pose3d> drawList) {

        int count = 0;
        for (int col = 0; col < 5 && count < gamePieceCount; col++) {
            for (int row = 0; row < 5 && count < gamePieceCount; row++) {
                count++;
                if (isBlue) {
                    drawList.add(new Pose3d(
                            blueRenderPose.plus(
                                    new Translation3d(Inches.of(-6 * col), Inches.of(6 * row), Inches.of(1.3 * col))),
                            new Rotation3d()));
                } else {
                    drawList.add(new Pose3d(
                            redRenderPose.plus(
                                    new Translation3d(Inches.of(6 * col), Inches.of(-6 * row), Inches.of(1.3 * col))),
                            new Rotation3d()));
                }
            }
        }
    }

    public void reset() {
        gamePieceCount = 24;
    }

    public void throwForGoal() {
        throwAtPoint(
                isBlue ? Rotation2d.fromDegrees(45) : Rotation2d.fromDegrees(220),
                Degrees.of(75),
                MetersPerSecond.of(11.2));
    }

    public void dump() {
        for (int i = 0; i < 24 && gamePieceCount > 0; i++) {
            gamePieceCount--;
            this.arena.addPieceWithVariance(
                    isBlue ? blueDumpPose.toTranslation2d() : redDumpPose.toTranslation2d(),
                    new Rotation2d(),
                    Meters.of(1.7),
                    isBlue ? MetersPerSecond.of(2) : MetersPerSecond.of(-2),
                    Degrees.of(0),
                    0,
                    0.2,
                    5,
                    0.2,
                    5.0);
        }
    }

    public void throwAtPoint(Rotation2d yaw, Angle pitch, LinearVelocity speed) {
        if (gamePieceCount > 0) {
            gamePieceCount--;
            arena.addPieceWithVariance(
                    isBlue ? blueLaunchPose.toTranslation2d() : redLaunchPose.toTranslation2d(),
                    yaw,
                    Meters.of(1.7),
                    speed,
                    pitch,
                    0,
                    0,
                    15,
                    2,
                    5);
        }
    }
}
