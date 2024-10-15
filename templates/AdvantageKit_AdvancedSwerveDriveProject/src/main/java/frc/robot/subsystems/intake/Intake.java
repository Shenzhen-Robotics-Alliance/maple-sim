package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The intake subsystem to exemplify intake simulation */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private IntakeInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeInputsAutoLogged();
    setDefaultCommand(this.run(() -> io.runIntakeVoltage(0)));
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** run the intake until the note goes into the intake */
  public Command runIntakeUntilNoteDetected() {
    return this.run(() -> io.runIntakeVoltage(6.0))
        .until(() -> inputs.noteDetected)
        .finallyDo(() -> io.runIntakeVoltage(0.0));
  }

  /**
   * launches a note from the intake, will run at full power to pass the note to the flywheels when
   * the note is gone for 0.3 seconds, the intake stops
   */
  public Command launchNote() {
    return this.run(() -> io.runIntakeVoltage(12))
        .raceWith(Commands.waitUntil(() -> !inputs.noteDetected).andThen(Commands.waitSeconds(0.3)))
        .finallyDo(() -> io.runIntakeVoltage(0.0));
  }

  /**
   * splits the note from the intake by running the intake reversely when the note is gone for 0.3
   * seconds, the intake stops
   */
  public Command splitNote() {
    return this.run(() -> io.runIntakeVoltage(-12))
        .raceWith(Commands.waitUntil(() -> !inputs.noteDetected).andThen(Commands.waitSeconds(0.3)))
        .finallyDo(() -> io.runIntakeVoltage(0.0));
  }
}
