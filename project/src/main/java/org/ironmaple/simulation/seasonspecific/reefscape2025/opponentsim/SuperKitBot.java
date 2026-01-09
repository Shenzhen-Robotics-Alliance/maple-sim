package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Objects;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.opponentsim.SmartOpponentConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class SuperKitBot extends SmartOpponent {
    /**
     * A SmartOpponent for FRC Reefscape. This robot breaks rules, hence "Super" KitBot.
     *
     * @param name the opponent name. Typically just "SuperKitBot 1".
     *             Names should not be the same.
     * @param alliance the opponents {@link edu.wpi.first.wpilibj.DriverStation.Alliance}.
     */
    public SuperKitBot(String name, DriverStation.Alliance alliance) {
        /// All Options should be set in the constructor.
        super(new SmartOpponentConfig()
                .withName(name)
                .withAlliance(alliance)
                .withManager(Arena2025Reefscape.getInstance().getOpponentManager())
                .withScoringPoseWeights("Net", 10)
                .withChassisConfig(SmartOpponentConfig.ChassisConfig.Presets.SimpleSquareChassis.getConfig()
                        .withBumperHeight(Inches.of(38))
                        .withBumperWidth(Inches.of(28))
                        .withTrackLength(Inches.of(28))
                        .withTrackWidth(Inches.of(18)))
                .withAutoEnable());
        /// Adds a new separate subsystem for the manipulator, this allows better manual control support.
        this.manipulatorSim
                /// Adds an intake to the manipulatorSim; you can add as many as you want with
                // .addIntakeSimulation(String name, IntakeSimulation)
                .addIntakeSimulation(
                        "Intake",
                        // Our new intake sim.
                        IntakeSimulation.InTheFrameIntake(
                                // This one focuses on coral.
                                "Coral",
                                // Our driveSim so the intake can tell if we collected a piece.
                                drivetrainSim.getDriveTrainSimulation(),
                                // How long the intake is from the center of the selected side.
                                Inches.of(20),
                                // Which side of the robot the intake is on.
                                IntakeSimulation.IntakeSide.FRONT,
                                // Max game piece capacity.
                                1))
                .addProjectileSimulation(
                        "Algae",
                        () -> new ReefscapeAlgaeOnFly(
                                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(), // Opponent Pose
                                new Translation2d(Inches.of(0), Inches.of(-18)), // Pose on bot
                                drivetrainSim.getActualSpeedsRobotRelative(), // Robot Relative opponent speeds
                                drivetrainSim.getActualPoseInSimulationWorld().getRotation(), // Shooter heading
                                Meters.of(2.5), // Shooter Height
                                MetersPerSecond.of(2), // Shooter speed
                                Degrees.of(20))) // Shooter angle
                .addProjectileSimulation("Reef L1", () -> newCoral(Meters.of(.7)))
                .addProjectileSimulation("Reef L2", () -> newCoral(Meters.of(1)))
                .addProjectileSimulation("Reef L3", () -> newCoral(Meters.of(1.5)))
                .addProjectileSimulation("Reef L4", () -> newCoral(Meters.of(3), Degrees.of(-75)));

                System.out.println("Projectile Simulation Started: " + manipulatorSim.getProjectileSimulations().toString());
    }

    /**
     * Creates a new CoralOnTheFly with static variables for the reef.
     *
     * @param height initial coral height.
     * @return the new {@link ReefscapeCoralOnFly}.
     */
    private ReefscapeCoralOnFly newCoral(Distance height) {
        return newCoral(height, Degrees.of(-15));
    }

    /**
     * Creates a new CoralOnTheFly with static variables for the reef.
     *
     * @param height initial coral height.
     * @param angle initial coral angle.
     * @return the new {@link ReefscapeCoralOnFly}.
     */
    private ReefscapeCoralOnFly newCoral(Distance height, Angle angle) {
        return new ReefscapeCoralOnFly(
                // Obtain robot position from drive simulation
                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(),
                // The scoring mechanism is installed at (x, y) (meters) on the robot
                new Translation2d(Inches.of(20), Inches.of(0)),
                // Obtain robot speed from drive simulation
                drivetrainSim.getActualSpeedsRobotRelative(),
                // Obtain robot facing from drive simulation
                drivetrainSim.getActualPoseInSimulationWorld().getRotation(),
                // The height at which the coral is ejected
                height,
                // The initial speed of the coral
                MetersPerSecond.of(2),
                // The coral is ejected at a 35-degree slope
                angle);
    }

    /**
     * The collect state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command collectState() {
        final Pair<String, Pose2d> target = getCollectTarget();
        if (target == null) return Commands.none();
        return pathfind(target, MetersPerSecond.of(8))
                .andThen(manipulatorSim.intake("Intake").withTimeout(0.5))
                .finallyDo(() -> setState("Score"))
                .withTimeout(12);
    }

    /**
     * Gets a random reef level to score.
     *
     * @return a random reef level to score.
     */
    private Command scoreReef() {
        // Random integer from 0-4;
        final int target = (int) (Math.random() * 4);
        return switch (target) {
            case 1 -> manipulatorSim.score("Reef L2");
            case 2 -> manipulatorSim.score("Reef L3");
            case 3 -> manipulatorSim.score("Reef L4");
            default -> manipulatorSim.score("Reef L1");
        };
    }

    /**
     * The score state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command scoreState() {
        final Pair<String, Pose2d> target = getScoringTarget();
        if (target == null) return Commands.none();
        return pathfind(target, MetersPerSecond.of(8))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(isCoralTarget() ? scoreReef() : manipulatorSim.score("Algae"))
                .andThen(Commands.waitSeconds(0.2))
                .finallyDo(() -> setState("Collect"))
                .withTimeout(12);
    }

    /**
     * Whether the current target is coral.
     *
     * @return whether the current target is coral.
     */
    private boolean isCoralTarget() {
        return !Objects.equals(target.getFirst(), "Barge Net");
    }

    /**
     * SuperKitBot Specific joystick method. Has button bindings too.
     *
     * @param xboxController the controller to use.
     * @return this, for chaining.
     */
    public SuperKitBot withXboxController(CommandXboxController xboxController) {
        config.withJoystick(xboxController);
        config.withState("Joystick", this::joystickState);
        config.withBehavior(
                "Player",
                startingState("Joystick").andThen(startingState("Joystick").ignoringDisable(true)));
        config.updateBehaviorChooser();
        /// Enable Manipulator control
        xboxController.leftBumper().and(config.isStateTrigger("Joystick")).whileTrue(manipulatorSim.intake("Intake"));
        xboxController.rightBumper().and(config.isStateTrigger("Joystick")).onTrue(manipulatorSim.score("Reef L1"));
        xboxController.b().and(config.isStateTrigger("Joystick")).onTrue(manipulatorSim.score("Reef L2"));
        xboxController.x().and(config.isStateTrigger("Joystick")).onTrue(manipulatorSim.score("Reef L3"));
        xboxController.y().and(config.isStateTrigger("Joystick")).onTrue(manipulatorSim.score("Reef L4"));
        xboxController.a().and(config.isStateTrigger("Joystick")).onTrue(manipulatorSim.score("Algae"));
        return this;
    }

    /**
     * The joystick state to run. Should be inaccessible when not set.
     *
     * @return the joystick state to run.
     */
    private Command joystickState() {
        final CommandXboxController xbox = ((CommandXboxController) config.getJoystick());
        return drive(
                () -> new ChassisSpeeds(
                        MathUtil.applyDeadband(
                                xbox.getLeftY() * -config.chassis.maxLinearVelocity.in(MetersPerSecond),
                                config.joystickDeadband),
                        MathUtil.applyDeadband(
                                xbox.getLeftX() * -config.chassis.maxLinearVelocity.in(MetersPerSecond),
                                config.joystickDeadband),
                        MathUtil.applyDeadband(
                                xbox.getRightX() * -config.chassis.maxAngularVelocity.in(RadiansPerSecond),
                                config.joystickDeadband)),
                false);
    }
}
