package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

/**
 * Example intake simulation, implementing IntakeIO notes can be stored in the intake, and when voltage is continuously
 * applied
 */
public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;
    private final AbstractDriveTrainSimulation driveTrain;
    private final Runnable passNoteToFlyWheelsCall;
    private double intakeVoltage = 0.0,
            // This is an indefinite integral of the intake motor voltage since the note has been in the
            // intake.
            // This approximates the position of the note in the intake.
            intakeVoltageIntegralSinceNoteTaken = 0.0;

    /**
     * @param driveTrain the swerve drivetrain simulation to which this intake is attached
     * @param passNoteToFlyWheelsCall called when the note in the intake is pushed to the flywheels, allowing the
     *     flywheels to simulate the projected note
     */
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain, Runnable passNoteToFlyWheelsCall) {
        this.intakeSimulation = new IntakeSimulation( // create intake simulation with no extension
                "Note", // the intake grabs game pieces of this type
                driveTrain, // specify the drivetrain to which the intake is attached to
                0.6, // the width of the intake
                IntakeSimulation.IntakeSide.BACK, // the intake is attached the back of the drivetrain
                1 // the intake can only hold 1 game piece at a time
                );
        intakeSimulation.register();

        this.driveTrain = driveTrain;
        this.passNoteToFlyWheelsCall = passNoteToFlyWheelsCall;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // gamePiecesInIntakeCount shows the amount of game pieces in the intake, we store this in the
        // inputs
        inputs.noteDetected = intakeSimulation.getGamePiecesAmount() != 0;

        // if the intake voltage is higher than 2 volts, it is considered running
        if (intakeVoltage > 4) intakeSimulation.startIntake();
        // otherwise, it's stopped
        else intakeSimulation.stopIntake();

        // if the there is note, we do an integral to the voltage to approximate the position of the
        // note in the intake
        if (inputs.noteDetected) intakeVoltageIntegralSinceNoteTaken += 0.02 * intakeVoltage;
        // if the note is gone, we clear the integral
        else intakeVoltageIntegralSinceNoteTaken = 0.0;

        // if the integral is negative, we get rid of the note
        if (intakeVoltageIntegralSinceNoteTaken < 0 && intakeSimulation.obtainGamePieceFromIntake())
            // splits the note out by adding it on field
            SimulatedArena.getInstance()
                    .addGamePiece(new CrescendoNoteOnField(driveTrain
                            .getSimulatedDriveTrainPose()
                            .getTranslation()
                            .plus(new Translation2d(-0.4, 0)
                                    .rotateBy(driveTrain
                                            .getSimulatedDriveTrainPose()
                                            .getRotation()))));
        // if the intake have been running positive volts since the note touches the intake, it will
        // touch the fly wheels
        else if (intakeVoltageIntegralSinceNoteTaken > 12 * 0.1 && intakeSimulation.obtainGamePieceFromIntake())
            // launch the note by calling the shoot note call back
            passNoteToFlyWheelsCall.run();
    }

    @Override
    public void runIntakeVoltage(double volts) {
        this.intakeVoltage = volts;
    }
}
