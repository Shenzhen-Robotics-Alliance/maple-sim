package org.ironmaple.simulation.gamepieces;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.LegacyFieldMirroringUtils2024;

/**
 *
 *
 * <h1>Simulates a Game Piece Launched into the Air</h1>
 *
 * <p>Check<a href='https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/'>Online
 * Documentation</a>
 *
 * <p>The movement is modeled by simple projectile motion.
 *
 * <p>If the projectile flies off the field, touches the ground, or hits its target, it will be automatically removed.
 *
 * <h3>Additional Features:</h3>
 *
 * <ul>
 *   <li>Optionally, it can be configured to become a {@link GamePieceOnFieldSimulation} upon touching the ground.
 *   <li>Optionally, it can be configured to have a "desired target." Upon hitting the target, it can be configured to
 *       run a callback.
 * </ul>
 *
 * <h3>Limitations:</h3>
 *
 * <ul>
 *   <li>Air drag is ignored.
 *   <li><strong>DOES NOT</strong> have collision space when flying.
 * </ul>
 */
public class GamePieceProjectile implements GamePieceInterface {
    /**
     * This value may seem unusual compared to the standard 9.8 m/s² for gravity. However, through experimentation, it
     * appears more realistic in our simulation, possibly due to the ignoring of air drag.
     */
    public static final double GRAVITY = 11;

    // Properties of the game piece projectile:
    protected final GamePieceOnFieldSimulation.GamePieceInfo info;
    public final String gamePieceType;
    protected final Translation2d initialPosition;
    protected final Translation2d initialLaunchingVelocityMPS;
    protected final double initialHeight, initialVerticalSpeedMPS;
    protected final Rotation3d gamePieceRotation;
    protected final Timer launchedTimer;

    /**
     *
     *
     * <h2>Visualizes the Projectile Flight Trajectory.</h2>
     *
     * <p>Optionally, this callback will be used to visualize the projectile flight trajectory in a telemetry system,
     * such as <a href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>.
     */
    private Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBackHitTarget = projectileTrajectory -> {};

    private Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBackMiss = projectileTrajectory -> {};

    // Optional properties of the game piece, used if we want it to become a
    // GamePieceOnFieldSimulation upon touching ground:
    protected boolean becomesGamePieceOnGroundAfterTouchGround = false;

    // Optional properties of the game piece, used if we want it to have a target:
    private Translation3d tolerance = new Translation3d(0.2, 0.2, 0.2);
    private Supplier<Translation3d> targetPositionSupplier = () -> new Translation3d(0, 0, -100);
    private Runnable hitTargetCallBack = () -> {};
    private double heightAsTouchGround = 0.5;

    /**
     *
     *
     * <h2>Time to Hit the Desired Target.</h2>
     *
     * <p>This value represents the amount of time it takes for the projectile to hit the desired target, calculated
     * when the {@link #launch()} method is called.
     *
     * <p>Determines the results of {@link #hasHitTarget()} and {@link #willHitTarget()}
     *
     * <p>If the projectile never hits the target, or if there is no target, this value remains <code>
     * -1</code>.
     */
    private double calculatedHitTargetTime = -1;

    private boolean hitTargetCallBackCalled = false;

    /**
     *
     *
     * <h2>Creates a Game Piece Projectile Ejected from a Shooter.</h2>
     *
     * @param info the info of the game piece
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisSpeedsFieldRelative the field-relative velocity of the robot chassis when launching the game piece,
     *     influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingSpeed the speed at which the game piece is launche
     * @param shooterAngle the pitch angle of the shooter when launching
     */
    public GamePieceProjectile(
            GamePieceOnFieldSimulation.GamePieceInfo info,
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeedsFieldRelative,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        this(
                info,
                robotPosition.plus(shooterPositionOnRobot.rotateBy(shooterFacing)),
                calculateInitialProjectileVelocityMPS(
                        shooterPositionOnRobot,
                        chassisSpeedsFieldRelative,
                        shooterFacing,
                        launchingSpeed.in(MetersPerSecond) * Math.cos(shooterAngle.in(Radians))),
                initialHeight.in(Meters),
                launchingSpeed.in(MetersPerSecond) * Math.sin(shooterAngle.in(Radians)),
                new Rotation3d(0, -shooterAngle.in(Radians), shooterFacing.getRadians()));
    }

    /**
     *
     *
     * <h2>Calculates the Initial Velocity of the Game Piece Projectile in the X-Y Plane.</h2>
     *
     * <p>This method calculates the initial velocity of the game piece projectile, accounting for the chassis's
     * translational and rotational motion as well as the shooter's ground speed.
     *
     * @param shooterPositionOnRobot the translation of the shooter on the robot, in the robot's frame of reference
     * @param chassisSpeeds the speeds of the chassis when the game piece is launched, including translational and
     *     rotational velocities
     * @param chassisFacing the direction the chassis is facing at the time of the launch
     * @param groundSpeedMPS the ground component of the projectile's initial velocity, provided as a scalar in meters
     *     per second (m/s)
     * @return the calculated initial velocity of the projectile as a {@link Translation2d} in meters per second
     */
    private static Translation2d calculateInitialProjectileVelocityMPS(
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d chassisFacing,
            double groundSpeedMPS) {
        final Translation2d
                chassisTranslationalVelocity =
                        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
                shooterGroundVelocityDueToChassisRotation =
                        shooterPositionOnRobot
                                .rotateBy(chassisFacing)
                                .rotateBy(Rotation2d.fromDegrees(90))
                                .times(chassisSpeeds.omegaRadiansPerSecond),
                shooterGroundVelocity = chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

        return shooterGroundVelocity.plus(new Translation2d(groundSpeedMPS, chassisFacing));
    }

    /**
     *
     *
     * <h2>Creates a Game Piece Projectile Ejected from a Shooter.</h2>
     *
     * @param info the info of the game piece
     * @param initialPosition the position of the game piece at the moment it is launched into the air
     * @param initialLaunchingVelocityMPS the horizontal component of the initial velocity in the X-Y plane, in meters
     *     per second (m/s)
     * @param initialHeight the initial height of the game piece when launched (the height of the shooter from the
     *     ground)
     * @param initialVerticalSpeedMPS the vertical component of the initial velocity, in meters per second (m/s)
     * @param gamePieceRotation the 3D rotation of the game piece during flight (only affects visualization of the game
     *     piece)
     */
    public GamePieceProjectile(
            GamePieceOnFieldSimulation.GamePieceInfo info,
            Translation2d initialPosition,
            Translation2d initialLaunchingVelocityMPS,
            double initialHeight,
            double initialVerticalSpeedMPS,
            Rotation3d gamePieceRotation) {
        this.info = info;
        this.gamePieceType = info.type();
        this.initialPosition = initialPosition;
        this.initialLaunchingVelocityMPS = initialLaunchingVelocityMPS;
        this.initialHeight = initialHeight;
        this.initialVerticalSpeedMPS = initialVerticalSpeedMPS;
        this.gamePieceRotation = gamePieceRotation;
        this.launchedTimer = new Timer();
    }

    /**
     *
     *
     * <h2>Starts the Game Piece Projectile Simulation.</h2>
     *
     * <p>This method initiates the projectile motion of the game piece with the following actions:
     *
     * <ul>
     *   <li>Initiates the projectile motion of the game piece. The current pose can be obtained with
     *       {@link #getPose3d()}.
     *   <li>Calculates whether the projectile will hit the target during its flight. The result can be obtained using
     *       {@link #willHitTarget()}.
     *   <li>Calculates a preview trajectory by simulating the projectile's motion for up to 100 steps, with each step
     *       lasting 0.02 seconds.
     *   <li>If specified, displays the trajectory using
     *       {@link GamePieceProjectile#projectileTrajectoryDisplayCallBackHitTarget}, which can be set via
     *       {@link GamePieceProjectile#withProjectileTrajectoryDisplayCallBack(Consumer)}.
     *   <li>Starts the {@link #launchedTimer}, which stores the amount of time elapsed after the game piece is launched
     * </ul>
     */
    public void launch() {
        final int maxIterations = 100;
        final double stepSeconds = 0.02;
        List<Pose3d> trajectoryPoints = new ArrayList<>();

        for (int i = 0; i < maxIterations; i++) {
            final double t = i * stepSeconds;
            final Translation3d currentPosition = getPositionAtTime(t);
            trajectoryPoints.add(new Pose3d(currentPosition, gamePieceRotation));

            if (currentPosition.getZ() < heightAsTouchGround && t * GRAVITY > initialVerticalSpeedMPS) break;
            if (isOutOfField(t)) break;
            final Translation3d displacementToTarget =
                    targetPositionSupplier.get().minus(currentPosition);
            if (Math.abs(displacementToTarget.getX()) < tolerance.getX()
                    && Math.abs(displacementToTarget.getY()) < tolerance.getY()
                    && Math.abs(displacementToTarget.getZ()) < tolerance.getZ()) {
                this.calculatedHitTargetTime = t;
                break;
            }
        }
        if (willHitTarget()) projectileTrajectoryDisplayCallBackHitTarget.accept(trajectoryPoints);
        else projectileTrajectoryDisplayCallBackMiss.accept(trajectoryPoints);
        this.hitTargetCallBackCalled = false;

        launchedTimer.start();
    }

    /**
     *
     *
     * <h2>Checks if the Game Piece Has Touched the Ground.</h2>
     *
     * <p>This method determines whether the game piece has touched the ground at the current time.
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     * </ul>
     *
     * @return <code>true</code> if the game piece has touched the ground, otherwise <code>false
     *     </code>
     */
    public boolean hasHitGround() {
        return getPositionAtTime(launchedTimer.get()).getZ() <= heightAsTouchGround
                && launchedTimer.get() * GRAVITY > initialVerticalSpeedMPS;
    }

    /**
     *
     *
     * <h2>Checks if the Game Piece Has Flown Out of the Field's Boundaries.</h2>
     *
     * <p>This method determines whether the game piece has flown out of the field's boundaries (outside the fence).
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     * </ul>
     *
     * @return <code>true</code> if the game piece has flown out of the field's boundaries, otherwise <code>false</code>
     */
    public boolean hasGoneOutOfField() {
        return isOutOfField(launchedTimer.get());
    }

    private boolean isOutOfField(double time) {
        final Translation3d position = getPositionAtTime(time);
        final double EDGE_TOLERANCE = 0.5;
        return position.getX() < -EDGE_TOLERANCE
                || position.getX() > LegacyFieldMirroringUtils2024.FIELD_WIDTH + EDGE_TOLERANCE
                || position.getY() < -EDGE_TOLERANCE
                || position.getY() > LegacyFieldMirroringUtils2024.FIELD_HEIGHT + EDGE_TOLERANCE;
    }

    /**
     *
     *
     * <h2>Checks if the projectile will hit the target <strong>AT SOME MOMENT</strong> during its flight</h2>
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     * </ul>
     *
     * <p>This is different from {@link #hasHitTarget()}
     */
    public boolean willHitTarget() {
        return calculatedHitTargetTime != -1;
    }

    /**
     *
     *
     * <h2>Checks if the Projectile Has Already Hit the Target <strong>At the Moment</strong>.</h2>
     *
     * <p>This method checks whether the projectile has hit the target at the current time.
     *
     * <ul>
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.
     *   <li>This is different from {@link #willHitTarget()}, which predicts whether the projectile will eventually hit
     *       the target.
     * </ul>
     *
     * @return <code>true</code> if the projectile has hit the target at the current time, otherwise <code>false</code>
     */
    public boolean hasHitTarget() {
        return willHitTarget() && launchedTimer.get() >= calculatedHitTargetTime;
    }

    /**
     *
     *
     * <h2>Clean up the trajectory through {@link #projectileTrajectoryDisplayCallBackHitTarget}</h2>
     *
     * @return this instance
     */
    public GamePieceProjectile cleanUp() {
        this.projectileTrajectoryDisplayCallBackHitTarget.accept(new ArrayList<>());
        this.projectileTrajectoryDisplayCallBackMiss.accept(new ArrayList<>());
        return this;
    }

    /**
     *
     *
     * <h2>Calculates the Projectile's Position at a Given Time.</h2>
     *
     * <p>This method calculates the position of the projectile using the physics formula for projectile motion.
     *
     * @param t the time elapsed after the launch of the projectile, in seconds
     * @return the calculated position of the projectile at time <code>t</code> as a {@link Translation3d} object
     */
    protected Translation3d getPositionAtTime(double t) {
        final double height = initialHeight + initialVerticalSpeedMPS * t - 1.0 / 2.0 * GRAVITY * t * t;

        final Translation2d current2dPosition = initialPosition.plus(initialLaunchingVelocityMPS.times(t));
        return new Translation3d(current2dPosition.getX(), current2dPosition.getY(), height);
    }

    /**
     *
     *
     * <h2>Calculates the Projectile's Velocity at a Given Time.</h2>
     *
     * <p>This method calculates the 3d velocity of the projectile using the physics formula for projectile motion.
     *
     * @param t the time elapsed after the launch of the projectile, in seconds
     * @return a {@link Translation3d} object representing the calculated 3d velocity of the projectile at time <code>t
     *     </code>, in meters per second
     */
    private Translation3d getVelocityMPSAtTime(double t) {
        final double verticalVelocityMPS = initialVerticalSpeedMPS - GRAVITY * t;

        return new Translation3d(
                initialLaunchingVelocityMPS.getX(), initialLaunchingVelocityMPS.getY(), verticalVelocityMPS);
    }

    /**
     *
     *
     * <h2>Calculates the Projectile's Current Position.</h2>
     *
     * <p>The position is calculated using {@link #getPositionAtTime(double)} while the rotation is pre-stored.
     *
     * @return a {@link Pose3d} object representing the current pose of the game piece
     */
    public Pose3d getPose3d() {
        return new Pose3d(getPositionAtTime(launchedTimer.get()), gamePieceRotation);
    }

    /**
     *
     *
     * <h2>Calculates the Projectile's Velocity at a Given Time.</h2>
     *
     * @see #getVelocityMPSAtTime(double)
     * @return a {@link Translation3d} object representing the calculated 3d velocity of the projectile at time <code>t
     *     </code>, in meters per second
     */
    public Translation3d getVelocity3dMPS() {
        return getVelocityMPSAtTime(launchedTimer.get());
    }

    /**
     *
     *
     * <h2>Adds a {@link GamePieceOnFieldSimulation} to a {@link SimulatedArena} to Simulate the Game Piece After
     * Touch-Ground.</h2>
     *
     * <p>The added {@link GamePieceOnFieldSimulation} will have the initial velocity of the game piece projectile.
     *
     * <p>The game piece will start falling from mid-air until it touches the ground.
     *
     * <p>The added {@link GamePieceOnFieldSimulation} will always have collision space on the field, even before
     * touching the ground.
     *
     * @param simulatedArena the arena simulation to which the game piece will be added, usually obtained from
     *     {@link SimulatedArena#getInstance()}
     */
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!becomesGamePieceOnGroundAfterTouchGround) return;
        simulatedArena.addGamePiece(new GamePieceOnFieldSimulation(
                info,
                () -> Math.max(
                        info.gamePieceHeight().in(Meters) / 2,
                        getPositionAtTime(launchedTimer.get()).getZ()),
                new Pose2d(getPositionAtTime(launchedTimer.get()).toTranslation2d(), new Rotation2d()),
                initialLaunchingVelocityMPS));
    }

    /**
     *
     *
     * <h2>Check every {@link GamePieceProjectile} instance for available actions.</h2>
     *
     * <p>1. If a game piece {@link #hasHitTarget()}, remove it and run {@link #hitTargetCallBack} specified by
     * {@link #withHitTargetCallBack(Runnable)}
     *
     * <p>2. If a game piece {@link #hasHitGround()}, remove it and create a corresponding
     * {@link GamePieceOnFieldSimulation} using {@link #addGamePieceAfterTouchGround(SimulatedArena)}
     *
     * <p>3. If a game piece {@link #hasGoneOutOfField()}, remove it.
     */
    public static void updateGamePieceProjectiles(
            SimulatedArena simulatedArena, Set<GamePieceProjectile> gamePieceProjectiles) {
        final Queue<GamePieceProjectile> toRemoves = new ArrayBlockingQueue<>(5);
        for (GamePieceProjectile gamePieceProjectile : gamePieceProjectiles) {
            if (gamePieceProjectile.hasHitTarget()
                    || gamePieceProjectile.hasHitGround()
                    || gamePieceProjectile.hasGoneOutOfField()) toRemoves.offer(gamePieceProjectile);
            if (gamePieceProjectile.hasHitTarget() && !gamePieceProjectile.hitTargetCallBackCalled) {
                gamePieceProjectile.hitTargetCallBack.run();
                gamePieceProjectile.hitTargetCallBackCalled = true;
            }
            if (gamePieceProjectile.hasHitGround()) gamePieceProjectile.addGamePieceAfterTouchGround(simulatedArena);
        }

        while (!toRemoves.isEmpty())
            gamePieceProjectiles.remove(toRemoves.poll().cleanUp());
    }

    // The rest are methods to configure a game piece projectile simulation
    /**
     *
     *
     * <h2>Configures the Game Piece Projectile to Automatically Become a {@link GamePieceOnFieldSimulation} Upon
     * Touching Ground.</h2>
     *
     * <p>This method configures the game piece projectile to transform into a {@link GamePieceOnFieldSimulation} when
     * it touches the ground.
     *
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile enableBecomesGamePieceOnFieldAfterTouchGround() {
        this.becomesGamePieceOnGroundAfterTouchGround = true;
        return this;
    }

    /**
     *
     *
     * <h2>Configures the Game Piece Projectile to Disappear Upon Touching Ground.</h2>
     *
     * <p>Reverts the effect of {@link #enableBecomesGamePieceOnFieldAfterTouchGround()}.
     */
    public GamePieceProjectile disableBecomesGamePieceOnFieldAfterTouchGround() {
        this.becomesGamePieceOnGroundAfterTouchGround = false;
        return this;
    }

    /**
     *
     *
     * <h2>Sets a Target for the Game Projectile.</h2>
     *
     * <p>Configures the {@link #targetPositionSupplier} of this game piece projectile.
     *
     * <p>The method {@link #launch()} will estimate whether or not the game piece will hit the target.
     *
     * <p>After calling {@link #launch()}, {@link #hasHitTarget()} will indicate whether the game piece has already hit
     * the target.
     *
     * <p>Before calling this method, the target position is <code>0, 0, -100 (x,y,z)</code>, which the projectile will
     * never hit.
     *
     * @param targetPositionSupplier the position of the target, represented as a {@link Translation3d}
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile withTargetPosition(Supplier<Translation3d> targetPositionSupplier) {
        this.targetPositionSupplier = targetPositionSupplier;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the Target Tolerance for the Game Projectile.</h2>
     *
     * <p>Configures the {@link #tolerance} for determining whether the game piece has hit the target. The tolerance
     * defines how close the projectile needs to be to the target for it to be considered a hit.
     *
     * <p>If this method is not called, the default tolerance is <code>0.2, 0.2, 0.2 (x,y,z)</code>
     *
     * @param tolerance the tolerance for the target, represented as a {@link Translation3d}
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile withTargetTolerance(Translation3d tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    /**
     *
     *
     * <h2>Configures a callback to be executed when the game piece hits the target.</h2>
     *
     * <p>Sets the {@link #hitTargetCallBack} to Execute When the Game Piece Hits the Target.
     *
     * <p>The callback will be triggered when {@link #hasHitTarget()} becomes <code>true</code>.
     *
     * @param hitTargetCallBack the callback to run when the game piece hits the target
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile withHitTargetCallBack(Runnable hitTargetCallBack) {
        this.hitTargetCallBack = hitTargetCallBack;
        return this;
    }

    /**
     *
     *
     * <h2>Configures a Callback to Display the Trajectory of the Projectile When Launched.</h2>
     *
     * <p>Sets the {@link #projectileTrajectoryDisplayCallBackHitTarget} to be fed with data during the
     * {@link #launch()} method.
     *
     * <p>A {@link List} containing up to 50 {@link Pose3d} objects will be passed to the callback, representing the
     * future trajectory of the projectile.
     *
     * <p>This is usually for visualizing the trajectory of the projectile on a telemetry, like <a
     * href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>
     *
     * @param projectileTrajectoryDisplayCallBack the callback that will receive the list of {@link Pose3d} objects
     *     representing the projectile's trajectory
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile withProjectileTrajectoryDisplayCallBack(
            Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBack) {
        this.projectileTrajectoryDisplayCallBackMiss =
                this.projectileTrajectoryDisplayCallBackHitTarget = projectileTrajectoryDisplayCallBack;
        return this;
    }

    /**
     *
     *
     * <h2>Configures a Callback to Display the Trajectory of the Projectile When Launched.</h2>
     *
     * <p>Sets the {@link #projectileTrajectoryDisplayCallBackHitTarget} to be fed with data during the
     * {@link #launch()} method.
     *
     * <p>A {@link List} containing up to 50 {@link Pose3d} objects will be passed to the callback, representing the
     * future trajectory of the projectile.
     *
     * <p>This is usually for visualizing the trajectory of the projectile on a telemetry, like <a
     * href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>
     *
     * @param projectileTrajectoryDisplayCallBackHitTarget the callback that will receive the list of {@link Pose3d}
     *     objects representing the projectile's trajectory, called if the projectile will hit the target on its path
     * @param projectileTrajectoryDisplayCallBackHitTargetMiss the callback that will receive the list of {@link Pose3d}
     *     objects representing the projectile's trajectory, called if the projectile will be off the target
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile withProjectileTrajectoryDisplayCallBack(
            Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBackHitTarget,
            Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBackHitTargetMiss) {
        this.projectileTrajectoryDisplayCallBackHitTarget = projectileTrajectoryDisplayCallBackHitTarget;
        this.projectileTrajectoryDisplayCallBackMiss = projectileTrajectoryDisplayCallBackHitTargetMiss;
        return this;
    }

    /**
     *
     *
     * <h2>Configures the Height at Which the Projectile Is Considered to Be Touching Ground.</h2>
     *
     * <p>Sets the {@link #heightAsTouchGround}, defining the height at which the projectile is considered to have
     * landed.
     *
     * <p>When the game piece is below this height, it will either be deleted or, if configured, transformed into a
     * {@link GamePieceOnFieldSimulation} using {@link #enableBecomesGamePieceOnFieldAfterTouchGround()}.
     *
     * @param heightAsTouchGround the height (in meters) at which the projectile is considered to touch the ground
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     */
    public GamePieceProjectile withTouchGroundHeight(double heightAsTouchGround) {
        this.heightAsTouchGround = heightAsTouchGround;
        return this;
    }

    @Override
    public String getType() {
        return this.gamePieceType;
    }

    @Override
    public boolean isGrounded() {
        return false;
    }
}
