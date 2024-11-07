// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;

import org.ironmaple.simulation.SimRobot;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Simulations are store in the robot container
    private final SwerveDriveSimulation swerveDriveSimulation;

    // Subsystems
    private final Drive drive;
    private final Flywheel flywheel;
    private final Intake intake;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                /* Real robot, instantiate hardware IO implementations */

                /* Disable Simulations */
                this.swerveDriveSimulation = null;

                /* Subsystems */
                // drive = new Drive(
                //         new GyroIOPigeon2(false),
                //         new ModuleIOSparkMax(0),
                //         new ModuleIOSparkMax(1),
                //         new ModuleIOSparkMax(2),
                //         new ModuleIOSparkMax(3));
                // flywheel = new Flywheel(new FlywheelIOSparkMax());
                drive = new Drive(
                        new GyroIOPigeon2(true),
                        new ModuleIOTalonFX(0),
                        new ModuleIOTalonFX(1),
                        new ModuleIOTalonFX(2),
                        new ModuleIOTalonFX(3));
                flywheel = new Flywheel(new FlywheelIOTalonFX());
                this.intake = new Intake(new IntakeIOTalonSRX());
                break;

            case SIM:
                /* Sim robot, instantiate physics sim IO implementations */

                final SimRobot simRobot = SimulatedArena.getInstance().robot();

                simRobot.initializeDrivetrain(SwerveConfig.Default());
                this.swerveDriveSimulation = simRobot.getDriveTrain();

                simRobot.initializeGamePieceStorage(
                    new Transform3d(
                        new Translation3d(0.0, 0.0, 0.5),
                        Rotation3d.kZero
                    )
                );

                // reset the field for auto (placing game-pieces in positions)
                SimulatedArena.getInstance().resetFieldForAuto();

                drive = new Drive(
                        new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                        new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                        new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                        new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                        new ModuleIOSim(swerveDriveSimulation.getModules()[3])
                );

                /* other subsystems are created with hardware simulation IOs */
                final FlywheelIOSim flywheelIOSim = new FlywheelIOSim(simRobot);
                flywheel = new Flywheel(flywheelIOSim);

                this.intake = new Intake(
                        new IntakeIOSim(
                                simRobot,
                                flywheelIOSim::receiveNote));
                // simulation
                break;

            default:
                /* Replayed robot, disable IO implementations */

                /* physics simulations are also not needed */
                this.swerveDriveSimulation = null;
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                flywheel = new Flywheel(new FlywheelIO() {
                });

                this.intake = new Intake((inputs) -> {
                });
                break;
        }

        // Set up auto routines
        NamedCommands.registerCommand(
                "Run Flywheel",
                Commands.startEnd(
                        () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
                        .withTimeout(5.0));
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Forward)",
                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Reverse)",
                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        this.swerveDriveSimulation == null
                                                ? new Pose2d(drive.getPose().getTranslation(), new Rotation2d())
                                                : swerveDriveSimulation.getSimulatedDriveTrainPose()),
                                drive)
                                .ignoringDisable(true));
        controller
                .a()
                .whileTrue(
                        Commands.startEnd(
                                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

        controller.leftTrigger(0.5).whileTrue(intake.runIntakeUntilNoteDetected());
        controller
                .rightBumper()
                .whileTrue(
                        new StartEndCommand(
                                () -> flywheel.runVelocity(3000), () -> flywheel.runVelocity(0), flywheel));
        controller.rightTrigger(0.5).and(controller.rightBumper()).whileTrue(intake.launchNote());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void updateSimulationField() {
        if (swerveDriveSimulation != null) {
            SimulatedArena.getInstance().simulationPeriodic();

            Logger.recordOutput(
                    "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());

            // final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
            // if (notes != null)
            //     Logger.recordOutput("FieldSimulation/Notes", notes.toArray(Pose3d[]::new));
        }

        intake.visualizeNoteInIntake(
                swerveDriveSimulation == null
                        ? drive.getPose()
                        : swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
}
