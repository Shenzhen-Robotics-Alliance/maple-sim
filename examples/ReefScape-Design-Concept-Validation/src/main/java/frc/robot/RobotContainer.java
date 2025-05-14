// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorShooter;
import frc.robot.subsystems.Intake;
import frc.robot.utils.DefenseRobotInSimulation;

public class RobotContainer {
    private final Drive drive;
    private final Intake intake;
    private final ElevatorShooter elevatorShooter;
    private final CommandXboxController driverXbox;
    private final XboxController defenserXbox;

    private final DefenseRobotInSimulation defenseRobot;

    public RobotContainer() {
        drive = new Drive();
        intake = new Intake(drive.driveSimulation.getDriveTrainSimulation());
        elevatorShooter = new ElevatorShooter(intake, drive.driveSimulation.getDriveTrainSimulation());
        driverXbox = new CommandXboxController(0);

        defenserXbox = new XboxController(1);
        defenseRobot = new DefenseRobotInSimulation(new Pose2d(2, 2, new Rotation2d()));
        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(drive.joystickDrive(
                () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX(), () -> -driverXbox.getRightX()));
        driverXbox.leftTrigger(0.5).whileTrue(intake.intakeCoralUntilDetected());
        driverXbox.x().onTrue(elevatorShooter.moveToHeight(0.0));
        driverXbox.a().onTrue(elevatorShooter.moveToHeight(0.65));
        driverXbox.b().onTrue(elevatorShooter.moveToHeight(1.05));

        defenseRobot.setDefaultCommand(defenseRobot.joystickDrive(
                () -> -defenserXbox.getLeftY(), () -> -defenserXbox.getLeftX(), () -> -defenserXbox.getRightX()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
