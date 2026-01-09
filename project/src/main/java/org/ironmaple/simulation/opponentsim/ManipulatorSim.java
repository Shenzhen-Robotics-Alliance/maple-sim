package org.ironmaple.simulation.opponentsim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
// TODO
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class ManipulatorSim extends SubsystemBase {
    /// Simulation Maps of saved manipulators
    private final Map<String, IntakeSimulation> intakeSimulations;
    private final Map<String, Supplier<GamePieceProjectile>> projectileSimulations;

    /** Creates a new manipulator simulation. */
    public ManipulatorSim() {
        this.intakeSimulations = new HashMap<>();
        this.projectileSimulations = new HashMap<>();
    }

    /**
     * The Map of {@link IntakeSimulation} types.
     *
     * @return a Map of {@link IntakeSimulation} types.
     */
    public Map<String, IntakeSimulation> getIntakeSimulations() {
        return intakeSimulations;
    }

    /**
     * The Map of the {@link GamePieceProjectile} types.
     *
     * @return a Map of the {@link GamePieceProjectile} types.
     */
    public Map<String, Supplier<GamePieceProjectile>> getProjectileSimulations() {
        return projectileSimulations;
    }

    /**
     * Adds an intake simulation to the manipulator simulation.
     *
     * @param intakeName The name of the intake simulation.
     * @param intakeSimulation The simulation to add.
     * @return this, for chaining.
     */
    public ManipulatorSim addIntakeSimulation(String intakeName, IntakeSimulation intakeSimulation) {
        this.intakeSimulations.put(intakeName, intakeSimulation);
        return this;
    }

    /**
     * Adds a projectile simulation to the manipulator simulation.
     *
     * @param projectileName The name of the projectile simulation.
     * @param projectileSimulation The simulation to add.
     * @return this, for chaining.
     */
    public ManipulatorSim addProjectileSimulation(
            String projectileName, Supplier<GamePieceProjectile> projectileSimulation) {
        this.projectileSimulations.put(projectileName, projectileSimulation);
        return this;
    }

    /**
     * Gets an intake simulation from the manipulator simulation.
     *
     * @param intakeName The name of the intake simulation.
     * @return The simulation.
     */
    public IntakeSimulation getIntakeSimulation(String intakeName) {
        return this.intakeSimulations.get(intakeName);
    }

    /**
     * Gets a projectile simulation from the manipulator simulation.
     *
     * @param projectileName The name of the simulation.
     * @return The simulation.
     */
    public Supplier<GamePieceProjectile> getProjectileSimulation(String projectileName) {
        return this.projectileSimulations.get(projectileName);
    }

    /**
     * Runs the intake, stops running the intake when the command ends.
     *
     * @param intakeName
     * @return
     */
    public Command intake(String intakeName) {
        return runEnd(() -> getIntakeSimulation(intakeName).startIntake(),
                () -> getIntakeSimulation(intakeName).stopIntake());
    }

    /**
     * Runs intake(String) until the game piece count in the intake goes up.
     *
     * @param intakeName
     * @return
     */
    public Command intakeUntilCollected(String intakeName) {
        final int count = getIntakeSimulation(intakeName).getGamePiecesAmount();
        return intake(intakeName)
                .until(() -> count > getIntakeSimulation(intakeName).getGamePiecesAmount());
    }

    /**
     * Adds a projectile to the simulation.
     *
     * @param projectileName The name of the projectile simulation.
     * @return a command to add the game piece projectile to the simulation.
     */
    public Command score(String projectileName) {
        return runOnce(() -> SimulatedArena.getInstance()
                .addGamePieceProjectile(getProjectileSimulation(projectileName).get()));
    }

    /**
     * A command that only calls score(String) when there is greater than zero(>0)
     * {@link org.ironmaple.simulation.gamepieces.GamePiece} in the intake.
     *
     * @param intakeName the intake to check.
     * @param projectileName the piece to score.
     * @return score(String) when valid.
     */
    public Command scoreWithIntake(String intakeName, String projectileName) {
        return score(projectileName)
                .onlyIf(() -> getIntakeSimulation(intakeName).getGamePiecesAmount() > 0);
    }
}
