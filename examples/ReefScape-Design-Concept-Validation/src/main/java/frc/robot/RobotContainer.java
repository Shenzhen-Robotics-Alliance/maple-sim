// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    private final Drive drive;
    private final CommandXboxController driverXbox;

    public RobotContainer() {
        drive = new Drive();
        driverXbox = new CommandXboxController(0);
        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(drive.joystickDrive(
                () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX(), () -> -driverXbox.getRightX()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
