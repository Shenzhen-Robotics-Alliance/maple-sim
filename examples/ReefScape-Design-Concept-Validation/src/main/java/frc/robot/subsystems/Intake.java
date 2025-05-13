package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MechanismVisualizer;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class Intake extends SubsystemBase {
    public final IntakeSimulation intakeSimulation;
    private final AbstractDriveTrainSimulation driveSim;
    private final TrapezoidProfile profile;

    private static final double intakeState = -35.0;
    private static final double idleState = 75.0;
    private TrapezoidProfile.State state = new TrapezoidProfile.State(idleState, 0.0);
    private double coralPosition;
    private boolean turnedOn;

    public Intake(AbstractDriveTrainSimulation driveSim) {
        super("Intake");

        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Coral", driveSim, Meters.of(0.6), Meters.of(0.35), IntakeSimulation.IntakeSide.BACK, 1);
        this.driveSim = driveSim;
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(360.0, 720.0));
        coralPosition = 0.0;
        turnedOn = false;
    }

    public void startIntake() {
        turnedOn = true;
    }

    public void stopIntake() {
        turnedOn = false;
    }

    public boolean hasCoral() {
        return intakeSimulation.getGamePiecesAmount() >= 1 && coralPosition > 0.8;
    }

    public Command intakeCoralUntilDetected() {
        return runEnd(this::startIntake, this::stopIntake).until(this::hasCoral);
    }

    public Command indexCoral(ElevatorShooter elevatorShooter) {
        return runEnd(this::startIntake, this::stopIntake);
    }

    private static final double coralIndexingSeconds = 0.6;

    @Override
    public void periodic() {
        if (turnedOn & intakeSimulation.getGamePiecesAmount() > 0) coralPosition += 0.02 / coralIndexingSeconds;

        double desiredPosition = turnedOn ? intakeState : idleState;
        state = profile.calculate(0.02, state, new TrapezoidProfile.State(desiredPosition, 0.0));
        if (state.position <= -25.0) intakeSimulation.startIntake();
        else intakeSimulation.stopIntake();

        MechanismVisualizer.getInstance().setIntakeAngle(Degrees.of(state.position));

        DogLog.log("Intake/CoralInIntake", intakeSimulation.getGamePiecesAmount() > 0 ? getCoralInIntakePose() : null);
    }

    /**
     * Visualizes the position of the coral inside the intake Uses a simple linear interpretation to display how much
     * the coral have been processed
     */
    private Pose3d getCoralInIntakePose() {
        //  Step0: Obtain drivetrain center pose from drive sim
        Pose3d driveTrainCenterPose = new Pose3d(driveSim.getSimulatedDriveTrainPose());

        // Step1: Calculate the pose of the coral IMMEDIATELY after being intake
        // Step1.1: Apply a twist to the drive center pose to get the coral pose
        Twist3d driveCenterToCoralStartingPose = new Twist3d(-0.55, 0.0, 0.15, 0.0, 0.0, 0.0);
        Pose3d coralStartingPose = driveTrainCenterPose.exp(driveCenterToCoralStartingPose);
        // Step1.2: Apply a rotation to the coral
        Transform3d rotation = new Transform3d(new Translation3d(), new Rotation3d(0.0, Math.toRadians(-30.0), 0.0));
        coralStartingPose = coralStartingPose.plus(rotation);

        // Step2: Calculate the pose of the coral after being COMPLETELY PROCESSED
        // (We use the same method as Step1).
        Twist3d driveCenterToCoralEndingPose = new Twist3d(0.14, 0.0, 0.22, 0.0, 0.0, 0.0);
        Pose3d coralEndingPose = driveTrainCenterPose.exp(driveCenterToCoralEndingPose);
        rotation = new Transform3d(new Translation3d(), new Rotation3d(0.0, Math.toRadians(17.5), 0.0));
        coralEndingPose = coralEndingPose.plus(rotation);

        // Step3: Find the current position of the coral using linear interpretation
        return coralStartingPose.interpolate(coralEndingPose, coralPosition);
    }

    public boolean obtainAvailableGamePiece() {
        boolean result = intakeSimulation.obtainGamePieceFromIntake();
        if (result) coralPosition = 0.0;
        return result;
    }
}
