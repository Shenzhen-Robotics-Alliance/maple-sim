package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.dyn4j.geometry.Convex;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Consumer;

/**
 * <h1>Simulates a game piece launched into the air</h1>
 * <p>The movement is modeled by a simple projectile motion. </p>
 * <p>If the projectile flies off-the field, touches ground or hits its target, it will be automatically removed</p>
 * <h3>Additional Features:</h3>
 * <p>1. Optionally, it can be configured to become a {@link GamePieceOnFieldSimulation} upon touch-ground</p>
 * <p>2. Optionally, it can be conferred to have a "desired target".  Upon hitting the target, it can be configured to run a call-back </p>
 * <h3>Limitations:</h3>
 * <p>1. Air drag is ignored</p>
 * <p>2. Does not have collision space.</p>
 * */
public class GamePieceProjectile {
    /*
     * Yeah, this makes no sense.
     * But it looks more realistic than 9.8m/s^2 for I don't why.
     * (Perhaps it's the air-drag)
     * */
    private static final double GRAVITY = 7.5;

    // Properties of the game piece projectile:
    public final String gamePieceType;
    private final Translation2d initialPosition, initialLaunchingVelocityMPS;
    private final double initialHeight, initialVerticalSpeedMPS;
    private final Rotation3d gamePieceRotation;
    private final Timer launchedTimer;

    // Optionally, this call back will be called to visualize the projectile flight trajectory to a telemetry, like AScope
    private Consumer<List<Pose3d>> activeProjectileTrajectoryDisplayCallBack = projectileTrajectory -> {};

    // Optional Properties of the game piece, if we want it to become a game-piece-on-field-simulation upon touch-ground
    private boolean becomesGamePieceOnGroundAfterTouchGround = false;
    private Convex shape = null;
    private double gamePieceHeight = 0.0, massKg = 0.0;

    // Optional Properties of the game piece, if we want it to have a target
    private Translation3d targetPosition = new Translation3d(-100, -100, -100), tolerance = new Translation3d(0.2 , 0.2, 0.2);
    private Runnable hitTargetCallBack = () -> {};
    private double heightAsTouchGround = 0.5;

    /*
    * The amount of time that it takes for the projectile to hit the desired target
    * Calculated upon calling launch() method
    * If it never hits the target or if there is no desired target at all, it remains -1
    * */
    private double calculatedHitTargetTime = -1;

    public GamePieceProjectile(String gamePieceType, Translation2d initialPosition, Translation2d shooterPositionOnRobot, ChassisSpeeds chassisSpeeds, Rotation2d chassisFacing, double initialHeight, double launchingSpeedMPS, double shooterAngleRad) {
        this(
                gamePieceType,
                initialPosition,
                calculateInitialLaunchSpeedMPS(
                        shooterPositionOnRobot, chassisSpeeds, chassisFacing,
                        launchingSpeedMPS * Math.cos(shooterAngleRad)
                ),
                initialHeight,
                launchingSpeedMPS * Math.sin(shooterAngleRad),
                new Rotation3d(0, -shooterAngleRad, chassisFacing.getRadians())
        );
    }

    private static Translation2d calculateInitialLaunchSpeedMPS(Translation2d shooterPositionOnRobot, ChassisSpeeds chassisSpeeds, Rotation2d chassisFacing, double groundSpeedMPS) {
        final Translation2d chassisTranslationalVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
                shooterGroundVelocityDueToChassisRotation = shooterPositionOnRobot
                        .rotateBy(chassisFacing)
                        .rotateBy(Rotation2d.fromDegrees(90))
                        .times(chassisSpeeds.omegaRadiansPerSecond),
                shooterGroundVelocity = chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

        return shooterGroundVelocity.plus(new Translation2d(groundSpeedMPS, chassisFacing));
    }

    public GamePieceProjectile(String gamePieceType, Translation2d initialPosition, Translation2d initialLaunchingVelocityMPS, double initialHeight, double initialVerticalSpeedMPS, Rotation3d gamePieceRotation) {
        this.gamePieceType = gamePieceType;
        this.initialPosition = initialPosition;
        this.initialLaunchingVelocityMPS = initialLaunchingVelocityMPS;
        this.initialHeight = initialHeight;
        this.initialVerticalSpeedMPS = initialVerticalSpeedMPS;
        this.gamePieceRotation = gamePieceRotation;
        this.launchedTimer = new Timer();
    }

    public void launch() {
        final int maxIterations = 100;
        final double stepSeconds = 0.02;
        List<Pose3d> trajectoryPoints = new ArrayList<>();

        for (int i = 0; i < maxIterations; i++) {
            final double t = i * stepSeconds;
            final Translation3d currentPosition = getPositionAtTime(t);
            trajectoryPoints.add(new Pose3d(currentPosition, gamePieceRotation));

            if (currentPosition.getZ() < 0)
                break;
            final Translation3d displacementToTarget = targetPosition.minus(currentPosition);
            if (Math.abs(displacementToTarget.getX()) < tolerance.getX()
                    && Math.abs(displacementToTarget.getX()) < tolerance.getX()
                    && Math.abs(displacementToTarget.getX()) < tolerance.getX()) {
                this.calculatedHitTargetTime = t;
            }
        }
        activeProjectileTrajectoryDisplayCallBack.accept(trajectoryPoints);

        launchedTimer.start();
    }

    public boolean hasHitGround() {
        return getPositionAtTime(launchedTimer.get()).getZ() <= heightAsTouchGround;
    }

    public boolean hasGoneOutOfField() {
        final Translation3d position = getPositionAtTime(launchedTimer.get());
        final double EDGE_TOLERANCE = 0.5;
        return position.getX() < -EDGE_TOLERANCE
                || position.getX() > FieldMirroringUtils.FIELD_WIDTH + EDGE_TOLERANCE
                || position.getY() < -EDGE_TOLERANCE
                || position.getY() > FieldMirroringUtils.FIELD_HEIGHT + EDGE_TOLERANCE;
    }

    public boolean willHitTarget() {
        return calculatedHitTargetTime != -1;
    }

    public boolean hasHitTarget() {
        return willHitTarget()
                && launchedTimer.get() >= calculatedHitTargetTime;
    }

    public boolean shouldBeRemoved() {
        return hasGoneOutOfField() || hasHitGround() || hasHitTarget();
    }

    public GamePieceProjectile cleanUp() {
        this.activeProjectileTrajectoryDisplayCallBack.accept(new ArrayList<>());
        return this;
    }

    private Translation3d getPositionAtTime(double t) {
        final double height = initialHeight + initialVerticalSpeedMPS * t - 1.0/2.0 * GRAVITY * t * t;

        final Translation2d current2dPosition = initialPosition.plus(initialLaunchingVelocityMPS.times(t));
        return new Translation3d(current2dPosition.getX(), current2dPosition.getY(), height);
    }

    public Pose3d getPose3d() {
        return new Pose3d(getPositionAtTime(launchedTimer.get()), gamePieceRotation);
    }

    public void onHitTarget() {
        this.hitTargetCallBack.run();
    }

    /**
     *
     * */
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!becomesGamePieceOnGroundAfterTouchGround)
            return;
        simulatedArena.addGamePiece(new GamePieceOnFieldSimulation(
                gamePieceType, shape,
                () -> Math.max(
                        gamePieceHeight/2,
                        getPositionAtTime(launchedTimer.get()).getZ()
                ),
                massKg,
                getPositionAtTime(launchedTimer.get()).toTranslation2d(),
                initialLaunchingVelocityMPS
        ));
    }

    public static void updateGamePieceProjectiles(SimulatedArena simulatedArena, Set<GamePieceProjectile> gamePieceProjectiles) {
        final Queue<GamePieceProjectile> toRemoves = new ArrayBlockingQueue<>(5);
        for (GamePieceProjectile gamePieceProjectile:gamePieceProjectiles) {
            if (gamePieceProjectile.shouldBeRemoved())
                toRemoves.offer(gamePieceProjectile);
            if (gamePieceProjectile.hasHitTarget())
                gamePieceProjectile.onHitTarget();
            if (gamePieceProjectile.hasHitGround())
                gamePieceProjectile.addGamePieceAfterTouchGround(simulatedArena);
        }

        while (!toRemoves.isEmpty())
            gamePieceProjectiles.remove(toRemoves.poll().cleanUp());
    }

    // The rest are methods to configure a game piece projectile simulation

    public GamePieceProjectile enableBecomesGamePieceOnFieldAfterTouchGround(Convex shape, double gamePieceHeightMeters, double massKg) {
        this.becomesGamePieceOnGroundAfterTouchGround = true;
        this.shape = shape;
        this.gamePieceHeight = gamePieceHeightMeters;
        this.massKg = massKg;
        return this;
    }

    public GamePieceProjectile withTargetPosition(Translation3d targetPosition) {
        this.targetPosition = targetPosition;
        return this;
    }

    public GamePieceProjectile withTargetTolerance(Translation3d tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public GamePieceProjectile withHitTargetCallBack(Runnable hitTargetCallBack) {
        this.hitTargetCallBack = hitTargetCallBack;
        return this;
    }

    public GamePieceProjectile withActiveProjectileTrajectoryDisplayCallBack(Consumer<List<Pose3d>> activeProjectileTrajectoryDisplayCallBack) {
        this.activeProjectileTrajectoryDisplayCallBack = activeProjectileTrajectoryDisplayCallBack;
        return this;
    }

    public GamePieceProjectile withTouchGroundHeight(double heightAsTouchGround) {
        this.heightAsTouchGround = heightAsTouchGround;
        return this;
    }
}
