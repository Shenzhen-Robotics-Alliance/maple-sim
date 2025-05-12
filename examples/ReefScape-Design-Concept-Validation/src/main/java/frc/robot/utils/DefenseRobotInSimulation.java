package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;

public class DefenseRobotInSimulation extends SubsystemBase {
    /* The drivetrain configuration for the opponent robots in the maple-sim simulation. */
    private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG = DriveTrainSimulationConfig.Default()
            .withRobotMass(Pounds.of(135))
            .withBumperSize(Inches.of(35), Inches.of(35))
            .withSwerveModule(COTS.ofMark4n(DCMotor.getKrakenX60Foc(1), DCMotor.getFalcon500(1), 1.8, 1));

    private final SelfControlledSwerveDriveSimulation driveSimulation;

    private static final Pose2d QUEUING_POSE = new Pose2d(-2, 5, new Rotation2d());

    public DefenseRobotInSimulation(Pose2d startingPoseBlue) {
        this.driveSimulation =
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(DRIVETRAIN_CONFIG, QUEUING_POSE));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
        RobotModeTriggers.disabled().onTrue(moveTo(QUEUING_POSE));
        RobotModeTriggers.teleop().onTrue(moveTo(FieldMirroringUtils.toCurrentAlliancePose(startingPoseBlue)));
    }

    @Override
    public void periodic() {
        // Assume odometry is perfectly accurate
        driveSimulation.resetOdometry(driveSimulation.getActualPoseInSimulationWorld());

        DogLog.log("Field/OpponentRobot", driveSimulation.getActualPoseInSimulationWorld());
    }

    public Command moveTo(Pose2d pose) {
        return runOnce(() -> driveSimulation.setSimulationWorldPose(pose)).finallyDo(this::stop);
    }

    public Command joystickDrive(
            DoubleSupplier driverXInput, DoubleSupplier driverYInput, DoubleSupplier driverZInput) {
        return run(() -> driveSimulation.runChassisSpeeds(
                        new ChassisSpeeds(
                                driverXInput.getAsDouble() * 4.0,
                                driverYInput.getAsDouble() * 4.0,
                                driverZInput.getAsDouble() * 2.0),
                        new Translation2d(),
                        true,
                        true))
                .finallyDo(this::stop);
    }

    public void stop() {
        driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false);
    }
}
