package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class Intake extends SubsystemBase {
    private final IntakeSimulation intakeSimulation;

    public Intake(AbstractDriveTrainSimulation driveTrainSimulation) {
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                "Note",
                driveTrainSimulation,
                Meters.of(0.7),
                IntakeSimulation.IntakeSide.BACK,
                1);
    }

    public Command runIntake() {
        return new FunctionalCommand(
                intakeSimulation::startIntake,
                () -> {},
                (interrupted) -> intakeSimulation.stopIntake(),
                () -> intakeSimulation.getGamePiecesAmount() > 0,
                this);
    }

    public Command clearGamePiece() {
        return Commands.runOnce(intakeSimulation::obtainGamePieceFromIntake, this);
    }
}
