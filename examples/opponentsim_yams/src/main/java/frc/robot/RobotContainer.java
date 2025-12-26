// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.evergreen.ArenaEvergreen;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.SuperKitBot;

public class RobotContainer
{
  // The YAMS swerve drive subsystem.
  private final SwerveSubsystem drive = new SwerveSubsystem();
  // Xbox Controller to drive the robot.
  private final CommandXboxController xboxController = new CommandXboxController(0);

  /// MapleSim GamePiece Pose Publishers
  private final StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
          .getTable("GamePieces")
          .getStructArrayTopic("Coral Array",
                  Pose3d.struct)
          .publish();
  private final StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
          .getTable("GamePieces")
          .getStructArrayTopic("Algae Array",
                  Pose3d.struct)
          .publish();

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
    // Set the YAM swerve drive subsystem to drive with the Xbox controller.
    drive.setDefaultCommand(drive.drive(drive.getChassisSpeedsSupplier(
            xboxController::getLeftY,
            xboxController::getLeftX,
            () -> xboxController.getRightX() * -1))); // Lamba expression to invert the right joystick axis.
    /// Enable breakdown publishing for MapleSim, and place game pieces on the field.
    SimulatedArena.getInstance().enableBreakdownPublishing();
    SimulatedArena.getInstance().resetFieldForAuto();
    configureBindings();

    /// Lower our subtick timing for slightly easier processing.
    SimulatedArena.overrideSimulationTimings(Seconds.of(0.02), 3);

    /// Let's add some opponents.
      // Since we are using opponent sim I want to disable battery sim since they statically share a battery.
      SimulatedBattery.disableBatterySim();
      new SuperKitBot("Super Bot 1", DriverStation.Alliance.Blue)
              .withXboxController(new CommandXboxController(1));
      new SuperKitBot("Super Bot 2", DriverStation.Alliance.Blue);
      new SuperKitBot("Super Bot 3", DriverStation.Alliance.Blue);
      new SuperKitBot("Super Bot 4", DriverStation.Alliance.Red);
      new SuperKitBot("Super Bot 5", DriverStation.Alliance.Red);
      new SuperKitBot("Super Bot 6", DriverStation.Alliance.Red);

  }

  private void configureBindings()
  {
    // Drive forward
    xboxController.button(1).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(0.5, 0, 0)));
    // Drive backward
    xboxController.button(2).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0)));
    // Drive to a Blue starting position
    xboxController.button(3).whileTrue(drive.driveToPose(new Pose2d(Meters.of(1.6),
                                                                    Meters.of(4),
                                                                    Rotation2d.fromDegrees(0))));
    // Drive to a Red starting position
    xboxController.button(4).whileTrue(drive.driveToPose(new Pose2d(Meters.of(-15),
                                                                    Meters.of(4),
                                                                    Rotation2d.fromDegrees(180))));
    // Zero Gyro and Reset Odometry
    xboxController.leftBumper().whileTrue(drive.zeroGyro());
    // Reset Odometry
    xboxController.rightBumper().onTrue(drive.resetOdometry(new Pose2d(1.5, 3, Rotation2d.fromDegrees(0))));
    // Drop coral at all stations at once.
    xboxController.start().onTrue(Commands.runOnce(() -> addCoralAllStations(true)));

  }

  /**
   * This method is called every simulation loop in {@link Robot#simulationPeriodic()}.
   * It is recommended to call these somewhere more appropriate in your code.
   */
  public void simulationPeriodic() {
      /// MapleSim must be updated every simulation loop, it doesn't matter where so long as it's called.
      SimulatedArena.getInstance().simulationPeriodic();
      /// Update MapleSim GamePiece Poses
      coralPoses.set(SimulatedArena.getInstance()
              .getGamePiecesArrayByType("Coral"));
      algaePoses.set(SimulatedArena.getInstance()
              .getGamePiecesArrayByType("Algae"));
  }

    /**
     * Just a basic method that uses in-built Maple-Sim functions.
     * Drops coral at all stations at once.
     *
     * @param isHorizontal whether the coral should be horizontal.
     */
    public static void addCoralAllStations(boolean isHorizontal) {
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.LEFT_STATION,
                DriverStation.Alliance.Blue,
                isHorizontal
        ));
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.RIGHT_STATION,
                DriverStation.Alliance.Blue,
                isHorizontal
        ));
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.LEFT_STATION,
                DriverStation.Alliance.Red,
                isHorizontal
        ));
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.RIGHT_STATION,
                DriverStation.Alliance.Red,
                isHorizontal
        ));
    }

  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}