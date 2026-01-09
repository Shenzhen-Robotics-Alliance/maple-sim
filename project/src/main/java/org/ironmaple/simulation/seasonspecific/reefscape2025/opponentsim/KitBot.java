package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.opponentsim.SmartOpponentConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class KitBot extends SmartOpponent {
    /**
     * A SmartOpponent for FRC Reefscape.
     *
     * @param name the opponent name. Typically just "SuperKitBot 1".
     *             Names should not be the same.
     * @param alliance the opponents {@link edu.wpi.first.wpilibj.DriverStation.Alliance}.
     */
    public KitBot(String name, DriverStation.Alliance alliance) {
        /// All Options should be set in the constructor.
        super(new SmartOpponentConfig()
                .withName(name)
                .withAlliance(alliance)
                .withManager(Arena2025Reefscape.getInstance().getOpponentManager())
                .withChassisConfig(SmartOpponentConfig.ChassisConfig.Presets.SimpleSquareChassis.getConfig()
                        .withMaxLinearVelocity(MetersPerSecond.of(4))
                        .withMaxAngularVelocity(DegreesPerSecond.of(360)))
                .removeScoringPoseType("Barge")
                .withAutoEnable());
        /// Adds a separate subsystem thread for the manipulator, this allows better manual control support.
        // TODO add better commenting
        this.manipulatorSim
                .addIntakeSimulation(
                        "Intake",
                        IntakeSimulation.InTheFrameIntake(
                                "Coral",
                                drivetrainSim.getDriveTrainSimulation(),
                                Inches.of(20),
                                IntakeSimulation.IntakeSide.FRONT,
                                1))
                .addProjectileSimulation(
                        "Coral",
                        () -> new ReefscapeCoralOnFly(
                                drivetrainSim.getActualPoseInSimulationWorld().getTranslation(),
                                new Translation2d(Inches.of(0), Inches.of(-18)), // Shooter on bot
                                drivetrainSim
                                        .getActualSpeedsFieldRelative()
                                        .plus(ChassisSpeeds.fromRobotRelativeSpeeds(
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
                                Degrees.of(0))); // Shooter angle
    }

    /**
     * The collect state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command collectState() {
        return pathfind(getCollectTarget(), MetersPerSecond.of(2))
                .andThen(manipulatorSim.intake("Intake").withTimeout(1.5))
                .finallyDo(() -> setState("Score"))
                .withTimeout(12);
    }

    /**
     * The score state to run.
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command scoreState() {
        return pathfind(getScoringTarget(), MetersPerSecond.of(2))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(manipulatorSim.score("Coral"))
                .andThen(Commands.waitSeconds(0.5))
                .finallyDo(() -> setState("Collect"))
                .withTimeout(12);
    }

    // TODO
    public KitBot withXboxController(CommandXboxController xboxController) {
        config.withJoystick(xboxController);
        config.withState("Joystick", this::joystickState);
        config.withBehavior(
                "Player",
                startingState("Joystick").andThen(startingState("Joystick").ignoringDisable(false)));
        config.updateBehaviorChooser();
        /// Enable Manipulator control
        xboxController.leftBumper().and(config.isStateTrigger("Joystick")).whileTrue(manipulatorSim.intake("Intake"));
        xboxController.rightBumper().and(config.isStateTrigger("Joystick")).whileTrue(manipulatorSim.score("Coral"));
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
