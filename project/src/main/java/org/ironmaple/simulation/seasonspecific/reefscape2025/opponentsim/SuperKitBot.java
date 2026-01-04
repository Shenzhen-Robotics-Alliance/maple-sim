package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    // TODO add commenr
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
                /// Adds a way to score. This specifies how the game piece leaves the robot to be scored.
                .addProjectileSimulation(
                        // We are scoring coral.
                        "Coral",
                        // The new projectile sim. This supplies the sim with the robot pose, so it must be a supplier
                        // of a new projectile.
                        () -> new ReefscapeCoralOnFly(
                                // Get the pose of the robot so we can score from it.
                                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(),
                                // Where the game piece should spawn relative to the robot.
                                new Translation2d(Inches.of(0), Inches.of(-18)),
                                /// Now we want this piece to leave the robot horizontally, so we have to do things a
                                // little differently.
                                // The robot speeds to help determine how fast the game piece should be when leaving the
                                // robot.
                                drivetrainSim
                                        .getActualSpeedsFieldRelative()
                                        // Now, to get our horizontal piece to fly correctly, we change the robot
                                        // speeds.
                                        .plus(ChassisSpeeds.fromRobotRelativeSpeeds(
                                                // Since our scorer it
                                                new ChassisSpeeds(
                                                        1, 0, 0), // Added chassis speeds to change coral velocity,
                                                drivetrainSim
                                                        .getActualPoseInSimulationWorld()
                                                        .getRotation())), // this is because we want a horizontal score
                                drivetrainSim
                                        .getActualPoseInSimulationWorld()
                                        .getRotation()
                                        .rotateBy(
                                                Rotation2d.kCCW_90deg), // Rotated by 90 degrees for horizontal shooter.
                                Inches.of(30), // Shooter Height
                                MetersPerSecond.of(0), // Initial piece speed
                                Degrees.of(0))) // Shooter angle
                .addProjectileSimulation(
                        "Algae",
                        () -> new ReefscapeAlgaeOnFly(
                                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(), // Opponent Pose
                                new Translation2d(Inches.of(0), Inches.of(-18)), // Pose on bot
                                drivetrainSim.getActualSpeedsRobotRelative(), // Robot Relative opponent speeds
                                drivetrainSim.getActualPoseInSimulationWorld().getRotation(), // Shooter heading
                                Meters.of(2.5), // Shooter Height
                                MetersPerSecond.of(2), // Shooter speed
                                Degrees.of(20))); // Shooter angle
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

        return pathfind(getCollectTarget())
                .andThen(manipulatorSim.intake("Intake").withTimeout(0.5))
                .finallyDo(() -> setState("Score"))
                .withTimeout(10);
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

        return pathfind(getScoringTarget())
                .andThen(Commands.waitSeconds(0.2))
                .andThen(isCoralTarget() ? manipulatorSim.score("Coral") : manipulatorSim.score("Algae"))
                .andThen(Commands.waitSeconds(0.5))
                .finallyDo(() -> setState("Collect"))
                .withTimeout(13);
    }

    // TODO
    private boolean isCoralTarget() {
        return !Objects.equals(target.getFirst(), "Barge Net");
    }

    // TODO
    public SuperKitBot withXboxController(CommandXboxController xboxController) {
        config.withJoystick(xboxController);
        config.withState("Joystick", this::joystickState);
        config.withBehavior(
                "Player",
                startingState("Joystick").andThen(startingState("Joystick").ignoringDisable(true)));
        config.updateBehaviorChooser();
        /// Enable Manipulator control
        xboxController.leftBumper().and(config.isStateTrigger("Joystick")).whileTrue(manipulatorSim.intake("Intake"));
        xboxController.rightBumper().and(config.isStateTrigger("Joystick")).onTrue(manipulatorSim.score("Coral"));
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
                                config.joystickdeadband),
                        MathUtil.applyDeadband(
                                xbox.getLeftX() * -config.chassis.maxLinearVelocity.in(MetersPerSecond),
                                config.joystickdeadband),
                        MathUtil.applyDeadband(
                                xbox.getRightX() * -config.chassis.maxAngularVelocity.in(RadiansPerSecond),
                                config.joystickdeadband)),
                false);
    }
}
