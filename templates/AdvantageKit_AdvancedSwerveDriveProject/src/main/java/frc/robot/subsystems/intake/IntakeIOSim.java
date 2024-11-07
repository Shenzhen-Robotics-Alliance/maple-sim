package frc.robot.subsystems.intake;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.ironmaple.simulation.GamePieceStorage;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimRobot;
import org.ironmaple.simulation.IntakeSimulation.IntakeBehavior;
import org.ironmaple.simulation.drivesims.DriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Note;

/**
 * Example intake simulation, implementing IntakeIO notes can be stored in the intake, and when
 * voltage is continuously applied
 */
public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSimulation;
  private final DriveTrainSimulation driveTrain;
  private final GamePieceStorage gamePieceStorage;
  private final Runnable passNoteToFlyWheelsCall;
  private double intakeVoltage = 0.0,
      // This is an indefinite integral of the intake motor voltage since the note has been in the
      // intake.
      // This approximates the position of the note in the intake.
      intakeVoltageIntegralSinceNoteTaken = 0.0;

  /**
   * @param driveTrain the swerve drivetrain simulation to which this intake is attached
   * @param passNoteToFlyWheelsCall called when the note in the intake is pushed to the flywheels,
   *     allowing the flywheels to simulate the projected note
   */
  public IntakeIOSim(SimRobot simRobot, Runnable passNoteToFlyWheelsCall) {
    this.intakeSimulation = simRobot.createIntake(
      new Pair<>(new Translation2d(-0.8, 0.8), new Translation2d(-1.1, -0.8)),
      IntakeBehavior.ADD_RANDOM, //lols
      Note.VARIANT
    );

    this.driveTrain = simRobot.getDriveTrain();
    this.gamePieceStorage = simRobot.getGamePieceStorage();
    this.passNoteToFlyWheelsCall = passNoteToFlyWheelsCall;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.noteDetected = gamePieceStorage.stored() > 0;

    // if the intake voltage is higher than 2 volts, it is considered running
    if (intakeVoltage > 4) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }

    // if the there is note, we do an integral to the voltage to approximate the position of the
    // note in the intake
    if (inputs.noteDetected) {
      intakeVoltageIntegralSinceNoteTaken += 0.02 * intakeVoltage;
    } else {
      intakeVoltageIntegralSinceNoteTaken = 0.0;
    }

    if (intakeVoltageIntegralSinceNoteTaken < 0 && inputs.noteDetected) {
      gamePieceStorage.pullLowest(true).ifPresent(note -> {
        Pose2d pose = driveTrain.getSimulatedDriveTrainPose();
        note.place(pose.getTranslation().plus(new Translation2d(-0.4, 0).rotateBy(pose.getRotation())));
      });
    } else if (intakeVoltageIntegralSinceNoteTaken > 12 * 0.1 && inputs.noteDetected) {
      passNoteToFlyWheelsCall.run();
    }
  }

  @Override
  public void runIntakeVoltage(double volts) {
    this.intakeVoltage = volts;
  }
}
