package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * An example of a simulated intake, which will such a note if activated.
 * */
public class IntakeExample extends IntakeSimulation {
  private final AbstractDriveTrainSimulation driveTrainSimulation;

  public IntakeExample(SimulatedArena arena, AbstractDriveTrainSimulation driveTrainSimulation) {
    super(arena, driveTrainSimulation, 0.6, IntakeSimulation.IntakeSide.BACK, 1);
    this.driveTrainSimulation = driveTrainSimulation;
  }

  @Override
  public void startIntake() {
    super.startIntake();
  }

  @Override
  public void stopIntake() {
    super.stopIntake();
  }

  /**
   * visualizes the note in the intake
   * */
  public void visualizeNoteInIntake() {
    final Pose3d robotPose = new Pose3d(driveTrainSimulation.getSimulatedDriveTrainPose()),
        noteInIntakePose =
            robotPose.plus(new Transform3d(0.1, 0, 0.4, new Rotation3d(0, -Math.toRadians(60), 0)));
    Logger.recordOutput(
        "Intake/NoteInIntake", this.gamePieceCount != 0 ? noteInIntakePose : new Pose3d());
  }

  public void clearGamePiece() {
    super.gamePieceCount = 0;
  }
}
