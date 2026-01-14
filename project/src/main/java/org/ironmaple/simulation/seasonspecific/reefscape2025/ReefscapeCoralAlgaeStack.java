package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h2>A CORAL-ALGAE stack on the field.</h2>
 *
 * <p>This class represents a CORAL-ALGAE stack, which consists of a standing CORAL with an ALGAE placed on top of it.
 *
 * <p>Three such stacks are staged on each side of the field before the match begins.
 *
 * <p>The stacks will collapse if hit, separating into a {@link ReefscapeCoralOnField} and a
 * {@link ReefscapeAlgaeOnField}.
 *
 * <p>Intakes can also grab a Coral and/or an Algae from the stack.
 *
 * <p>To visualize these stacks, use {@link #getStackedAlgaePoses()} and {@link #getStackedCoralPoses()} to retrieve the
 * positions of the coral and algae in the stacks, and display them through AScope.
 */
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

    private Translation2d velocityMPS() {
        return GeometryConvertor.toWpilibTranslation2d(getLinearVelocity());
    }

    private Rotation2d velocityDirection() {
        return velocityMPS().getAngle();
    }

    private Translation2d stackPosition() {
        return getPoseOnField().getTranslation();
    }

    private void collapse() {
        // remove the stack
        arena.removeGamePiece(this);

        throwCoralToGround();
        throwAlgaeToGround();

        // mark as collapsed
        collapsed = true;
    }

    private void throwCoralToGround() {
        ReefscapeCoralOnField coral = new ReefscapeCoralOnField(new Pose2d(
                stackPosition().plus(new Translation2d(0.15, 0).rotateBy(velocityDirection())), velocityDirection()));
        coral.setLinearVelocity(getLinearVelocity());
        arena.addGamePiece(coral);
    }

    private void throwAlgaeToGround() {
        arena.addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                stackPosition(),
                new Translation2d(),
                new ChassisSpeeds(),
                velocityDirection(),
                Meters.of(0.3).plus(Inches.of(8)),
                MetersPerSecond.of(velocityMPS().getNorm() * 0.6),
                Degrees.zero()));
    }

    /**
     *
     *
     * <h2>Retrieves the positions of the Coral pieces in all stacks.</h2>
     *
     * @return list of {@link Pose3d} representing the positions of the Coral pieces in all stacks in the arena
     */
    public static List<Pose3d> getStackedCoralPoses() {
        return getStackedCoralPoses(SimulatedArena.getInstance());
    }

    /** @see #getStackedCoralPoses() */
    private static final Transform3d STACK_TO_CORAL =
            new Transform3d(new Translation3d(0, 0, 0.15), new Rotation3d(0, Math.toRadians(90), 0));

    public static List<Pose3d> getStackedCoralPoses(SimulatedArena arena) {
        return arena.getGamePiecesPosesByType(REEFSCAPE_STACK_INFO.type()).stream()
                .map(stackPose -> stackPose.plus(STACK_TO_CORAL))
                .toList();
    }

    /**
     *
     *
     * <h2>Retrieves the positions of the Algae pieces in all stacks.</h2>
     *
     * @return a list of {@link Pose3d} representing the positions of the Algae pieces in all stacks in the arena
     */
    public static List<Pose3d> getStackedAlgaePoses() {
        return getStackedAlgaePoses(SimulatedArena.getInstance());
    }

    private static final Transform3d STACK_TO_ALGAE = new Transform3d(
            new Translation3d(0, 0, 0.3 + edu.wpi.first.math.util.Units.inchesToMeters(8)), new Rotation3d());

    /** @see #getStackedAlgaePoses() */
    public static List<Pose3d> getStackedAlgaePoses(SimulatedArena arena) {
        return arena.getGamePiecesPosesByType(REEFSCAPE_STACK_INFO.type()).stream()
                .map(stackPose -> stackPose.plus(STACK_TO_ALGAE))
                .toList();
    }

    @Override
    public void onIntake(String intakeTargetGamePieceType) {
        if ("Coral".equals(intakeTargetGamePieceType)) throwAlgaeToGround();
        else if ("Algae".equals(intakeTargetGamePieceType)) throwCoralToGround();
    }
}
