package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MechanismVisualizer;

public class ElevatorShooter extends SubsystemBase {
    public static final double MAX_ELEVATOR_HEIGHT = 1.1;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State currentState;
    private double goalMeters;
    private boolean hasCoral;

    public ElevatorShooter() {
        super("Elevator-Shooter");

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.6, 6.0));
        currentState = new TrapezoidProfile.State(0.0, 0.0);
        goalMeters = 0.0;
        hasCoral = true;
    }

    @Override
    public void periodic() {
        currentState = profile.calculate(0.02, currentState, new TrapezoidProfile.State(goalMeters, 0.0));
        MechanismVisualizer.getInstance().setElevatorHeight(Meters.of(getHeightMeters()));
    }

    public Command moveToHeight(double height) {
        return runOnce(() -> this.goalMeters = height)
                .andThen(Commands.waitUntil(() -> this.currentState.position == goalMeters));
    }

    public double getHeightMeters() {
        return currentState.position;
    }

    public Command score() {
        return runOnce(() -> {}).onlyIf(this::hasCoral);
    }

    public boolean hasCoral() {
        return hasCoral;
    }
}
