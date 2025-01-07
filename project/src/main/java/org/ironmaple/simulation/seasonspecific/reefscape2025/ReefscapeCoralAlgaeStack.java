package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;

public class ReefscapeCoralAlgaeStack extends GamePieceOnFieldSimulation {
    public static final GamePieceInfo REEFSCAPE_STACK_INFO =
            new GamePieceInfo("CoralAlgaeStack", new Circle(0.15), Meters.zero(), Kilograms.of(0.7), 1.8, 4, 0.3);

    private final SimulatedArena arena;
    private final Translation2d initialPosition;
    private boolean collapsed = false;

    public ReefscapeCoralAlgaeStack(Translation2d initialPosition) {
        this(initialPosition, SimulatedArena.getInstance());
    }

    public ReefscapeCoralAlgaeStack(Translation2d initialPosition, SimulatedArena arena) {
        super(REEFSCAPE_STACK_INFO, new Pose2d(initialPosition, new Rotation2d()));
        this.arena = arena;
        this.initialPosition = initialPosition;
        arena.addCustomSimulation(this::simulationSubTick);
    }

    private void simulationSubTick(int subTickCount) {
        // if already collapsed, skip
        if (collapsed) return;
        // make the stack collapse if its moved
        if (getPoseOnField().getTranslation().minus(initialPosition).getNorm() > 0.2
                && getLinearVelocity().getMagnitude() > 0.3) collapse();
    }

    private void collapse() {
        Translation2d velocityMPS = GeometryConvertor.toWpilibTranslation2d(getLinearVelocity());
        Rotation2d collapseDirection = velocityMPS.getAngle();
        Translation2d stackPosition = getPoseOnField().getTranslation();

        // remove the stack
        arena.removeGamePiece(this);

        // add the coral
        ReefscapeCoral coral = new ReefscapeCoral(new Pose2d(
                stackPosition.plus(new Translation2d(0.15, 0).rotateBy(collapseDirection)), collapseDirection));
        System.out.println("coral position: " + coral.getPoseOnField());
        coral.setLinearVelocity(getLinearVelocity());
        arena.addGamePiece(coral);

        // add the algae
        arena.addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                stackPosition,
                new Translation2d(),
                new ChassisSpeeds(),
                collapseDirection,
                0.3 + edu.wpi.first.math.util.Units.inchesToMeters(8),
                velocityMPS.getNorm() * 0.6,
                0));

        collapsed = true;
    }

    public static Pose3d[] getStackedCoralPoses() {
        return getStackedCoralPoses(SimulatedArena.getInstance());
    }

    private static final Transform3d STACK_TO_CORAL =
            new Transform3d(new Translation3d(0, 0, 0.15), new Rotation3d(0, Math.toRadians(90), 0));

    public static Pose3d[] getStackedCoralPoses(SimulatedArena arena) {
        return arena.getGamePiecesByType(REEFSCAPE_STACK_INFO.type()).stream()
                .map(stackPose -> stackPose.plus(STACK_TO_CORAL))
                .toArray(Pose3d[]::new);
    }

    public static Pose3d[] getStackedAlgaePoses() {
        return getStackedAlgaePoses(SimulatedArena.getInstance());
    }

    private static final Transform3d STACK_TO_ALGAE = new Transform3d(
            new Translation3d(0, 0, 0.3 + edu.wpi.first.math.util.Units.inchesToMeters(8)), new Rotation3d());

    public static Pose3d[] getStackedAlgaePoses(SimulatedArena arena) {
        return arena.getGamePiecesByType(REEFSCAPE_STACK_INFO.type()).stream()
                .map(stackPose -> stackPose.plus(STACK_TO_ALGAE))
                .toArray(Pose3d[]::new);
    }
}
