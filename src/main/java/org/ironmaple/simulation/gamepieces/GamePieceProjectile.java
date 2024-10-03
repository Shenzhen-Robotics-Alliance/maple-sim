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
 * <h1>Simulates a Game Piece Launched into the Air</h1>
 *
 * <p>The movement is modeled by simple projectile motion.</p>
 * <p>If the projectile flies off the field, touches the ground, or hits its target, it will be automatically removed.</p>
 *
 * <h3>Additional Features:</h3>
 * <ul>
 *   <li>Optionally, it can be configured to become a {@link GamePieceOnFieldSimulation} upon touching the ground.</li>
 *   <li>Optionally, it can be configured to have a "desired target." Upon hitting the target, it can be configured to run a callback.</li>
 * </ul>
 *
 * <h3>Limitations:</h3>
 * <ul>
 *   <li>Air drag is ignored.</li>
 *   <li><strong>DOES NOT</strong> have collision space when flying.</li>
 * </ul>
 */
public class GamePieceProjectile {
    /**
     * This value may seem unusual compared to the standard 9.8 m/s² for gravity.
     * However, through experimentation, it appears more realistic in our simulation, possibly due to the ignoring of air drag.
     * */
    private static final double GRAVITY = 8;

    // Properties of the game piece projectile:
    public final String gamePieceType;
    private final Translation2d initialPosition, initialLaunchingVelocityMPS;
    private final double initialHeight, initialVerticalSpeedMPS;
    private final Rotation3d gamePieceRotation;
    private final Timer launchedTimer;

    /**
     * <h2>Visualizes the Projectile Flight Trajectory.</h2>
     *
     * <p>Optionally, this callback will be used to visualize the projectile flight trajectory in a telemetry system, such as
     * <a href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a>.</p>
     * */
    private Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBack = projectileTrajectory -> {};

    // Optional properties of the game piece, used if we want it to become a GamePieceOnFieldSimulation upon touching ground:
    private boolean becomesGamePieceOnGroundAfterTouchGround = false;
    private Convex shape = null;
    private double gamePieceHeight = 0.0, massKg = 0.0;

    // Optional properties of the game piece, used if we want it to have a target:
    private Translation3d targetPosition = new Translation3d(0, 0, -100), tolerance = new Translation3d(0.2 , 0.2, 0.2);
    private Runnable hitTargetCallBack = () -> {};
    private double heightAsTouchGround = 0.5;

    /**
     * <h2>Time to Hit the Desired Target.</h2>
     *
     * <p>This value represents the amount of time it takes for the projectile to hit the desired target, calculated when the {@link #launch()} method is called.</p>
     * <p>Determines the results of {@link #hasHitTarget()} and {@link #willHitTarget()}</p>
     * <p>If the projectile never hits the target, or if there is no target, this value remains <code>-1</code>.</p>
     * */
    private double calculatedHitTargetTime = -1;
    private boolean hitTargetCallBackCalled = false;


    /**
     * <h2>Creates a Game Piece Projectile Ejected from a Shooter.</h2>
     *
     * @param gamePieceType the type of game piece, which will affect the {@link SimulatedArena#getGamePiecesByType(String)}
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's frame of reference
     * @param chassisSpeeds the velocity of the robot chassis when launching the game piece, influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the ground
     * @param launchingSpeedMPS the speed at which the game piece is launched, in meters per second (m/s)
     * @param shooterAngleRad the pitch angle of the shooter when launching, in radians
     * */
    public GamePieceProjectile(String gamePieceType, Translation2d robotPosition, Translation2d shooterPositionOnRobot, ChassisSpeeds chassisSpeeds, Rotation2d shooterFacing, double initialHeight, double launchingSpeedMPS, double shooterAngleRad) {
        this(
                gamePieceType,
                robotPosition.plus(shooterPositionOnRobot.rotateBy(shooterFacing)),
                calculateInitialProjectileVelocityMPS(
                        shooterPositionOnRobot, chassisSpeeds, shooterFacing,
                        launchingSpeedMPS * Math.cos(shooterAngleRad)
                ),
                initialHeight,
                launchingSpeedMPS * Math.sin(shooterAngleRad),
                new Rotation3d(0, -shooterAngleRad, shooterFacing.getRadians())
        );
    }

    /**
     * <h2>Calculates the Initial Velocity of the Game Piece Projectile in the X-Y Plane.</h2>
     *
     * <p>This method calculates the initial velocity of the game piece projectile, accounting for the chassis's translational and rotational motion as well as the shooter's ground speed.</p>
     *
     * @param shooterPositionOnRobot the translation of the shooter on the robot, in the robot's frame of reference
     * @param chassisSpeeds the speeds of the chassis when the game piece is launched, including translational and rotational velocities
     * @param chassisFacing the direction the chassis is facing at the time of the launch
     * @param groundSpeedMPS the ground component of the projectile's initial velocity, provided as a scalar in meters per second (m/s)
     * @return the calculated initial velocity of the projectile as a {@link Translation2d} in meters per second
     * */
    private static Translation2d calculateInitialProjectileVelocityMPS(Translation2d shooterPositionOnRobot, ChassisSpeeds chassisSpeeds, Rotation2d chassisFacing, double groundSpeedMPS) {
        final Translation2d chassisTranslationalVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
                shooterGroundVelocityDueToChassisRotation = shooterPositionOnRobot
                        .rotateBy(chassisFacing)
                        .rotateBy(Rotation2d.fromDegrees(90))
                        .times(chassisSpeeds.omegaRadiansPerSecond),
                shooterGroundVelocity = chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

        return shooterGroundVelocity.plus(new Translation2d(groundSpeedMPS, chassisFacing));
    }

    /**
     * <h2>Creates a Game Piece Projectile Ejected from a Shooter.</h2>
     *
     * @param gamePieceType the type of game piece, which will affect the {@link SimulatedArena#getGamePiecesByType(String)}
     * @param initialPosition the position of the game piece at the moment it is launched into the air
     * @param initialLaunchingVelocityMPS the horizontal component of the initial velocity in the X-Y plane, in meters per second (m/s)
     * @param initialHeight the initial height of the game piece when launched (the height of the shooter from the ground)
     * @param initialVerticalSpeedMPS the vertical component of the initial velocity, in meters per second (m/s)
     * @param gamePieceRotation the 3D rotation of the game piece during flight (only affects visualization of the game piece)
     * */
    public GamePieceProjectile(String gamePieceType, Translation2d initialPosition, Translation2d initialLaunchingVelocityMPS, double initialHeight, double initialVerticalSpeedMPS, Rotation3d gamePieceRotation) {
        this.gamePieceType = gamePieceType;
        this.initialPosition = initialPosition;
        this.initialLaunchingVelocityMPS = initialLaunchingVelocityMPS;
        this.initialHeight = initialHeight;
        this.initialVerticalSpeedMPS = initialVerticalSpeedMPS;
        this.gamePieceRotation = gamePieceRotation;
        this.launchedTimer = new Timer();
    }

    /**
     * <h2>Starts the Game Piece Projectile Simulation.</h2>
     *
     * <p>This method initiates the projectile motion of the game piece with the following actions:</p>
     * <ul>
     *   <li>Initiates the projectile motion of the game piece. The current pose can be obtained with {@link #getPose3d()}.</li>
     *   <li>Calculates whether the projectile will hit the target during its flight. The result can be obtained using {@link #willHitTarget()}.</li>
     *   <li>Calculates a preview trajectory by simulating the projectile's motion for up to 100 steps, with each step lasting 0.02 seconds.</li>
     *   <li>If specified, displays the trajectory using {@link GamePieceProjectile#projectileTrajectoryDisplayCallBack}, which can be set via {@link GamePieceProjectile#withProjectileTrajectoryDisplayCallBack(Consumer)}.</li>
     *   <li>Starts the {@link #launchedTimer}, which stores the amount of time elapsed after the game piece is launched</li>
     * </ul>
     * */
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
        projectileTrajectoryDisplayCallBack.accept(trajectoryPoints);
        this.hitTargetCallBackCalled = false;

        launchedTimer.start();
    }

    /**
     * <h2>Checks if the Game Piece Has Touched the Ground.</h2>
     *
     * <p>This method determines whether the game piece has touched the ground at the current time.</p>
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.</li>
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.</li>
     * </ul>
     *
     * @return <code>true</code> if the game piece has touched the ground, otherwise <code>false</code>
     * */
    public boolean hasHitGround() {
        return getPositionAtTime(launchedTimer.get()).getZ() <= heightAsTouchGround;
    }

    /**
     * <h2>Checks if the Game Piece Has Flown Out of the Field's Boundaries.</h2>
     *
     * <p>This method determines whether the game piece has flown out of the field's boundaries (outside the fence).</p>
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.</li>
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.</li>
     * </ul>
     *
     * @return <code>true</code> if the game piece has flown out of the field's boundaries, otherwise <code>false</code>
     * */
    public boolean hasGoneOutOfField() {
        final Translation3d position = getPositionAtTime(launchedTimer.get());
        final double EDGE_TOLERANCE = 0.5;
        return position.getX() < -EDGE_TOLERANCE
                || position.getX() > FieldMirroringUtils.FIELD_WIDTH + EDGE_TOLERANCE
                || position.getY() < -EDGE_TOLERANCE
                || position.getY() > FieldMirroringUtils.FIELD_HEIGHT + EDGE_TOLERANCE;
    }

    /**
     * <h2>Checks if the projectile will hit the target <strong>AT SOME MOMENT</strong> during its flight</h2>
     *
     * <ul>
     *   <li>The result is calculated during the {@link #launch()} method.</li>
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.</li>
     * </ul>
     *
     * <p>This is different from {@link #hasHitTarget()}</p>
     * */
    public boolean willHitTarget() {
        return calculatedHitTargetTime != -1;
    }

    /**
     * <h2>Checks if the Projectile Has Already Hit the Target <strong>At the Moment</strong>.</h2>
     *
     * <p>This method checks whether the projectile has hit the target at the current time.</p>
     *
     * <ul>
     *   <li>Before calling {@link #launch()}, this method will always return <code>false</code>.</li>
     *   <li>This is different from {@link #willHitTarget()}, which predicts whether the projectile will eventually hit the target.</li>
     * </ul>
     *
     * @return <code>true</code> if the projectile has hit the target at the current time, otherwise <code>false</code>
     * */
    public boolean hasHitTarget() {
        return willHitTarget()
                && launchedTimer.get() >= calculatedHitTargetTime;
    }

    /**
     * <h2>Clean up the trajectory through {@link #projectileTrajectoryDisplayCallBack}</h2>
     *
     * @return this instance
     * */
    public GamePieceProjectile cleanUp() {
        this.projectileTrajectoryDisplayCallBack.accept(new ArrayList<>());
        return this;
    }


    /**
     * <h2>Calculates the Projectile's Position at a Given Time.</h2>
     *
     * <p>This method calculates the position of the projectile using the physics formula for projectile motion.</p>
     *
     * @param t the time elapsed after the launch of the projectile, in seconds
     * @return the calculated position of the projectile at time <code>t</code> as a {@link Translation3d} object
     * */
    private Translation3d getPositionAtTime(double t) {
        final double height = initialHeight + initialVerticalSpeedMPS * t - 1.0/2.0 * GRAVITY * t * t;

        final Translation2d current2dPosition = initialPosition.plus(initialLaunchingVelocityMPS.times(t));
        return new Translation3d(current2dPosition.getX(), current2dPosition.getY(), height);
    }

    /**
     * <h2>Calculates the Projectile's Current Position.</h2>
     *
     * <p>The position is calculated using {@link #getPositionAtTime(double)} while the rotation is pre-stored.</p>
     *
     * @return a {@link Pose3d} object representing the current pose of the game piece
     * */
    public Pose3d getPose3d() {
        return new Pose3d(getPositionAtTime(launchedTimer.get()), gamePieceRotation);
    }

     /**
     * <h2>Adds a {@link GamePieceOnFieldSimulation} to a {@link SimulatedArena} to Simulate the Game Piece After Touch-Ground.</h2>
     *
     * <p>The added {@link GamePieceOnFieldSimulation} will have the initial velocity of the game piece projectile.</p>
     * <p>The game piece will start falling from mid-air until it touches the ground.</p>
     * <p>The added {@link GamePieceOnFieldSimulation} will always have collision space on the field, even before touching the ground.</p>
     *
     * @param simulatedArena the arena simulation to which the game piece will be added, usually obtained from {@link SimulatedArena#getInstance()}
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

    /**
     * <h2>Check every {@link GamePieceProjectile} instance for available actions.</h2>
     *
     * <p>1. If a game piece {@link #hasHitTarget()}, remove it and run {@link #hitTargetCallBack} specified by {@link #withHitTargetCallBack(Runnable)}</p>
     * <p>2. If a game piece {@link #hasHitGround()}, remove it and create a corresponding {@link GamePieceOnFieldSimulation} using {@link #addGamePieceAfterTouchGround(SimulatedArena)}</p>
     * <p>3. If a game piece {@link #hasGoneOutOfField()}, remove it. </p>
     * */
    public static void updateGamePieceProjectiles(SimulatedArena simulatedArena, Set<GamePieceProjectile> gamePieceProjectiles) {
        final Queue<GamePieceProjectile> toRemoves = new ArrayBlockingQueue<>(5);
        for (GamePieceProjectile gamePieceProjectile:gamePieceProjectiles) {
            if (gamePieceProjectile.hasHitTarget() || gamePieceProjectile.hasHitGround() || gamePieceProjectile.hasGoneOutOfField())
                toRemoves.offer(gamePieceProjectile);
            if (gamePieceProjectile.hasHitTarget() && !gamePieceProjectile.hitTargetCallBackCalled) {
                gamePieceProjectile.hitTargetCallBack.run();
                gamePieceProjectile.hitTargetCallBackCalled = true;
            }
            if (gamePieceProjectile.hasHitGround())
                gamePieceProjectile.addGamePieceAfterTouchGround(simulatedArena);
        }

        while (!toRemoves.isEmpty())
            gamePieceProjectiles.remove(toRemoves.poll().cleanUp());
    }

    // The rest are methods to configure a game piece projectile simulation
    /**
     * <h2>Configures the Game Piece Projectile to Automatically Become a {@link GamePieceOnFieldSimulation} Upon Touching Ground.</h2>
     *
     * <p>This method configures the game piece projectile to transform into a {@link GamePieceOnFieldSimulation} when it touches the ground.</p>
     *
     * @param shape the shape of the game piece's collision space
     * @param gamePieceHeightMeters the height (thickness) of the game piece, in meters
     * @param massKg the mass of the game piece, in kilograms
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     * */
    public GamePieceProjectile enableBecomesGamePieceOnFieldAfterTouchGround(Convex shape, double gamePieceHeightMeters, double massKg) {
        this.becomesGamePieceOnGroundAfterTouchGround = true;
        this.shape = shape;
        this.gamePieceHeight = gamePieceHeightMeters;
        this.massKg = massKg;
        return this;
    }

    /**
     * <h2>Sets a Target for the Game Projectile.</h2>
     *
     * <p>Configures the {@link #targetPosition} of this game piece projectile.</p>
     * <p>The method {@link #launch()} will estimate whether or not the game piece will hit the target.</p>
     * <p>After calling {@link #launch()}, {@link #hasHitTarget()} will indicate whether the game piece has already hit the target.</p>
     * <p>Before calling this method, the target position is <code>0, 0, -100 (x,y,z)</code>, which the projectile will never hit.</p>
     *
     * @param targetPosition the position of the target, represented as a {@link Translation3d}
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     * */
    public GamePieceProjectile withTargetPosition(Translation3d targetPosition) {
        this.targetPosition = targetPosition;
        return this;
    }

    /**
     * <h2>Sets the Target Tolerance for the Game Projectile.</h2>
     *
     * <p>Configures the {@link #tolerance} for determining whether the game piece has hit the target. The tolerance defines how close the projectile needs to be to the target for it to be considered a hit.</p>
     * <p>If this method is not called, the default tolerance is <code>0.2, 0.2, 0.2 (x,y,z)</code></p>
     *
     * @param tolerance the tolerance for the target, represented as a {@link Translation3d}
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     * */
    public GamePieceProjectile withTargetTolerance(Translation3d tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    /**
     * <h2>Configures a callback to be executed when the game piece hits the target.</h2>
     *
     * <p>Sets the {@link #hitTargetCallBack} to Execute When the Game Piece Hits the Target.</p>
     * <p>The callback will be triggered when {@link #hasHitTarget()} becomes <code>true</code>.</p>
     *
     * @param hitTargetCallBack the callback to run when the game piece hits the target
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     * */
    public GamePieceProjectile withHitTargetCallBack(Runnable hitTargetCallBack) {
        this.hitTargetCallBack = hitTargetCallBack;
        return this;
    }

    /**
     * <h2>Configures a Callback to Display the Trajectory of the Projectile When Launched.</h2>
     *
     * <p>Sets the {@link #projectileTrajectoryDisplayCallBack} to be fed with data during the {@link #launch()} method.</p>
     * <p>A {@link List} containing up to 50 {@link Pose3d} objects will be passed to the callback, representing the future trajectory of the projectile.</p>
     * <p>This is usually for visualizing the trajectory of the projectile on a telemetry, like <a href='https://github.com/Mechanical-Advantage/AdvantageScope'>Advantage Scope</a></p>
     *
     * @param projectileTrajectoryDisplayCallBack the callback that will receive the list of {@link Pose3d} objects representing the projectile's trajectory
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     * */
    public GamePieceProjectile withProjectileTrajectoryDisplayCallBack(Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBack) {
        this.projectileTrajectoryDisplayCallBack = projectileTrajectoryDisplayCallBack;
        return this;
    }

    /**
     * <h2>Configures the Height at Which the Projectile Is Considered to Be Touching Ground.</h2>
     *
     * <p>Sets the {@link #heightAsTouchGround}, defining the height at which the projectile is considered to have landed.</p>
     * <p>When the game piece is below this height, it will either be deleted or, if configured, transformed into a {@link GamePieceOnFieldSimulation} using {@link #enableBecomesGamePieceOnFieldAfterTouchGround(Convex, double, double)}.</p>
     *
     * @param heightAsTouchGround the height (in meters) at which the projectile is considered to touch the ground
     * @return the current instance of {@link GamePieceProjectile} to allow method chaining
     * */
    public GamePieceProjectile withTouchGroundHeight(double heightAsTouchGround) {
        this.heightAsTouchGround = heightAsTouchGround;
        return this;
    }
}
