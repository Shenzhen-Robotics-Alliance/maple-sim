package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.MechanismVisualizer.ELEVATOR_PITCH_ANGLE;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MechanismVisualizer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class ElevatorShooter extends SubsystemBase {
    public static final double MAX_ELEVATOR_HEIGHT = 1.1;
    private final TrapezoidProfile profile;
    private final Intake intake;
    private final AbstractDriveTrainSimulation driveSim;

    private TrapezoidProfile.State currentState;
    private double goalMeters;
    private boolean hasCoral;

    public ElevatorShooter(Intake intake, AbstractDriveTrainSimulation driveSim) {
        super("Elevator-Shooter");

        this.intake = intake;
        this.driveSim = driveSim;
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2.4, 12.0));
        currentState = new TrapezoidProfile.State(0.0, 0.0);
        goalMeters = 0.0;
        hasCoral = false;
    }

    @Override
    public void periodic() {
        currentState = profile.calculate(0.02, currentState, new TrapezoidProfile.State(goalMeters, 0.0));
        MechanismVisualizer.getInstance().setElevatorHeight(Meters.of(getHeightMeters()));

        if (currentState.position < 0.02 && !hasCoral) hasCoral = intake.obtainAvailableGamePiece();

        DogLog.log("Shooter/Coral", hasCoral ? getCoralInShooterPose() : new Pose3d(0, 0, -1, new Rotation3d()));
    }

    private Pose3d getCoralInShooterPose() {
        //  Step0: Obtain drivetrain center pose from drive sim
        Pose3d driveTrainCenterPose = new Pose3d(driveSim.getSimulatedDriveTrainPose());

        // Step1: Apply Twist to
        Twist3d driveCenterToCoralLowestPose = new Twist3d(0.02, 0.0, 0.25, 0.0, 0.0, 0.0);
        Pose3d coralPose = driveTrainCenterPose.exp(driveCenterToCoralLowestPose);

        // Step2: Add Elevator Height
        Twist3d elevatorHeight = new Twist3d(
                getHeightMeters() * Math.cos(ELEVATOR_PITCH_ANGLE.in(Radians)),
                0,
                getHeightMeters() * Math.sin(ELEVATOR_PITCH_ANGLE.in(Radians)),
                0.0,
                0.0,
                0.0);
        coralPose = coralPose.exp(elevatorHeight);

        // Step3: Rotate
        Transform3d rotation = new Transform3d(new Translation3d(), new Rotation3d(0.0, Math.toRadians(10), 0.0));
        return coralPose.plus(rotation);
    }

    public Command moveToHeight(double height) {
        return runOnce(() -> this.goalMeters = height)
                .andThen(Commands.waitUntil(() -> this.currentState.position == goalMeters));
    }

    public double getHeightMeters() {
        return currentState.position;
    }

    public Command score() {
        return runOnce(this::launchCoral).finallyDo(() -> hasCoral = false).onlyIf(this::hasCoral);
    }

    private void launchCoral() {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        driveSim.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(0.2, 0.0),
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSim.getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(getHeightMeters() + 0.25),
                        MetersPerSecond.of(3.5),
                        Degrees.of(-10)));
    }

    public boolean hasCoral() {
        return hasCoral;
    }
}
