package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * Visualizes the mechanism state to AdvantageScope
 * */
public class MechanismVisualizer {
    private static final Angle ELEVATOR_PITCH_ANGLE = Degrees.of(78);
    private static final Pose3d INTAKE_ZERO_POSE = new Pose3d(-0.26, 0, 0.278, new Rotation3d(0, -Math.toRadians(12), 0));
    private static final Pose3d ELEVATOR_ZERO_POSE = new Pose3d(0, 0, 0, new Rotation3d());

    private Distance elevatorHeight;
    private Angle intakeAngle;
    public MechanismVisualizer() {
        this.elevatorHeight = Meters.zero();
        this.intakeAngle = Degrees.of(90);
    }

    public void setElevatorHeight(Distance elevatorHeight) {
        this.elevatorHeight = elevatorHeight;
    }

    public void setIntakeAngle(Angle intakeAngle) {
        this.intakeAngle = intakeAngle;
    }

    public void displayMechanismPoseToAdvantageScope() {
        double pitchRad = ;
        Pose3d intakePose = INTAKE_ZERO_POSE.plus(new Transform3d(new Translation3d(), new Rotation3d(0, -intakeAngle.in(Radians), 0)));
        Pose3d secondSt
        Logger.recordOutput(
                "MechanismPoses",
                ,
                new Pose3d(extend * Math.cos(ELEVATOR_PITCH_ANGLE.in(Radians)) / 2, 0, extend * Math.sin(angle) / 2, new Rotation3d()),
                new Pose3d(extend * Math.cos(ELEVATOR_PITCH_ANGLE.in(Radians)), 0, extend * Math.sin(angle), new Rotation3d()));
    }

    private static MechanismVisualizer instance;
    public static MechanismVisualizer getInstance() {
        if (instance == null)
            instance = new MechanismVisualizer();

        return instance;
    }
}
