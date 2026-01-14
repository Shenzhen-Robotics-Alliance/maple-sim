package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.*;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.utils.FieldMirroringUtils;

/**
 *
 *
 * <h2>Simulates a <strong>HUB</strong>s on the field.</h2>
 *
 * <p>This class simulates the <strong>HUB</strong>s on the field where <strong>FUEL</strong>s can be scored. Whether it
 * is active can be determined by using {@link Arena2026Rebuilt#isActive(boolean)}
 */
public class RebuiltHub extends Goal {

    protected static final Translation3d blueHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
    protected static final Translation3d redHubPose = new Translation3d(11.938, 4.034536, 1.5748);
    protected static final Pose3d[] blueShootPoses = {
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, 0.447675, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(33.75))),
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, 0.149225, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(11.25))),
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, -0.149225, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(11.25))),
        new Pose3d(
                blueHubPose.plus(new Translation3d(0.5969, -0.447675, -0.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(-33.75)))
    };

    public static final double GoalRadius = 0.5969;
    static final Random rng = new Random();

    public static final Pose3d[] redShootPoses = Arrays.stream(blueShootPoses)
            .map(FieldMirroringUtils::flip)
            .map((Pose3d toRotate) -> {
                return toRotate.rotateBy(new Rotation3d(Rotation2d.fromDegrees(180)));
            })
            .toArray(Pose3d[]::new);

    StructPublisher<Pose3d> posePublisher;
    protected final Arena2026Rebuilt arena;

    /**
     *
     *
     * <h2>Creates an HUB of the specified color.</h2>
     *
     * @param arena The host arena of this HUB.
     * @param isBlue Wether this is the blue HUB or the red one.
     */
    public RebuiltHub(Arena2026Rebuilt arena, boolean isBlue) {
        super(
                arena,
                Inches.of(47),
                Inches.of(47),
                Inches.of(10),
                "Fuel",
                isBlue ? blueHubPose : redHubPose,
                isBlue,
                false);

        this.arena = arena;
        StructPublisher<Pose3d> HubPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(isBlue ? "BlueHub" : "RedHub", Pose3d.struct)
                .publish();
        HubPosePublisher.set(new Pose3d(position, new Rotation3d()));
    }

    @Override
    protected boolean checkVel(GamePiece gamePiece) {
        return gamePiece.getVelocity3dMPS().getZ() < 0;
    }

    @Override
    protected boolean checkCollision(GamePiece GamePiece) {
        return Math.pow(GamePiece.getPose3d().getX() - position.getX(), 2)
                        + Math.pow(GamePiece.getPose3d().getY() - position.getY(), 2)
                        + Math.pow(GamePiece.getPose3d().getZ() - position.getZ(), 2)
                < Math.pow(GoalRadius, 2);
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        arena.addValueToMatchBreakdown(isBlue, "WastedFuel", arena.isActive(isBlue) ? 0 : 1);
        arena.addToScore(isBlue, arena.isActive(isBlue) ? 1 : 0);

        Pose3d shootPose = isBlue ? blueShootPoses[rng.nextInt(4)] : redShootPoses[rng.nextInt(4)];

        arena.addPieceWithVariance(
                shootPose.getTranslation().toTranslation2d(),
                new Rotation2d(shootPose.getRotation().getZ()),
                shootPose.getMeasureZ(),
                MetersPerSecond.of(2),
                shootPose.getRotation().getMeasureY(),
                0,
                0.02,
                15,
                0.2,
                5);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        return;
    }
}
