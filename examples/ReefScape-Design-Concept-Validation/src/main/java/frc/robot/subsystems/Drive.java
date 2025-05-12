package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;

public class Drive extends SubsystemBase {
    public static final DriveTrainSimulationConfig simConfig =
            DriveTrainSimulationConfig.Default().withBumperSize(Inches.of(33), Inches.of(33));
    public static final RobotConfig ppConfig = new RobotConfig(
            Kilograms.of(45),
            KilogramSquareMeters.of(5.0),
            new ModuleConfig(Inches.of(2), FeetPerSecond.of(16.5), 1.2, DCMotor.getFalcon500(1), 6.75, Amps.of(40), 1),
            Meters.of(0.52));

    public final SelfControlledSwerveDriveSimulation driveSimulation;

    public Drive() {
        super("Drive");

        this.driveSimulation =
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(simConfig, new Pose2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    }

    @Override
    public void periodic() {
        // Assume odometry is perfectly accurate
        driveSimulation.resetOdometry(driveSimulation.getActualPoseInSimulationWorld());

        DogLog.log("Drive/RobotPose", driveSimulation.getActualPoseInSimulationWorld());
    }

    public void stop() {
        driveSimulation.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false);
    }

    public Command joystickDrive(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier zAxis) {
        double maxLinearVel = driveSimulation.maxLinearVelocity().in(MetersPerSecond);
        double maxAngularVel = driveSimulation.maxAngularVelocity().in(RadiansPerSecond);
        return run(() -> {
                    Translation2d linearVelocity =
                            new Translation2d(xAxis.getAsDouble(), yAxis.getAsDouble()).times(maxLinearVel);
                    double angularVelocity = maxAngularVel * zAxis.getAsDouble() * 0.6;

                    driveSimulation.runChassisSpeeds(
                            new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), angularVelocity),
                            new Translation2d(),
                            true,
                            true);
                    System.out.println("linear vel:" + linearVelocity);
                })
                .finallyDo(this::stop);
    }

    public Command followPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                driveSimulation::getActualPoseInSimulationWorld,
                driveSimulation::getActualSpeedsRobotRelative,
                (speeds, feedforwards) -> driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
                new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
                ppConfig,
                FieldMirroringUtils::isSidePresentedAsRed,
                this);
    }
}
