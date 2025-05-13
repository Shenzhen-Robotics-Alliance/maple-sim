package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Visualizes the mechanism state to AdvantageScope */
public class MechanismVisualizer {
    public static final Angle ELEVATOR_PITCH_ANGLE = Degrees.of(78);
    private static final Pose3d INTAKE_ZERO_POSE =
            new Pose3d(-0.26, 0, 0.278, new Rotation3d(0, -Math.toRadians(12), 0));

    private Distance elevatorHeight;
    private Angle intakeAngle;

    public MechanismVisualizer() {
        this.elevatorHeight = Meters.zero();
        this.intakeAngle = Degrees.of(-36);
    }

    public void setElevatorHeight(Distance elevatorHeight) {
        this.elevatorHeight = elevatorHeight;
    }

    public void setIntakeAngle(Angle intakeAngle) {
        this.intakeAngle = intakeAngle;
    }

    public void displayMechanismPoseToAdvantageScope() {
        double pitchRad = ELEVATOR_PITCH_ANGLE.in(Radians);
        double heightMeters = elevatorHeight.in(Meters);
        Pose3d intakePose = INTAKE_ZERO_POSE.plus(
                new Transform3d(new Translation3d(), new Rotation3d(0, intakeAngle.in(Radians), 0)));
        DogLog.log("MechanismPoses", new Pose3d[] {
            intakePose,
            new Pose3d(
                    heightMeters * Math.cos(pitchRad) / 2, 0, heightMeters * Math.sin(pitchRad) / 2, new Rotation3d()),
            new Pose3d(heightMeters * Math.cos(pitchRad), 0, heightMeters * Math.sin(pitchRad), new Rotation3d())
        });
    }

    private static MechanismVisualizer instance;

    public static MechanismVisualizer getInstance() {
        if (instance == null) instance = new MechanismVisualizer();

        return instance;
    }
}
