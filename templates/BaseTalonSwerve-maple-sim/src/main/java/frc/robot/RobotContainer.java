package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.MapleSimSwerve;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.TalonSwerve;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton resetRobotPose = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final SwerveDrive s_Swerve;
    private Intake intake = null;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        this.s_Swerve = Robot.isReal() ? new TalonSwerve() : new MapleSimSwerve();
        this.s_Swerve.initSwerveWidget("SwerveDrive");

        if (s_Swerve instanceof MapleSimSwerve simSwerve) this.intake = new Intake(simSwerve.getSimDrive());

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        s_Swerve.setDefaultCommand(new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                robotCentric));

        /* Driver Buttons */
        zeroGyro.onTrue(Commands.runOnce(s_Swerve::zeroHeading).ignoringDisable(true));

        final Command placeRobotPose = Commands.runOnce(() -> s_Swerve.setPose(new Pose2d(3, 3, new Rotation2d())));
        resetRobotPose.onTrue(placeRobotPose);

        if (intake != null) {
            new Trigger(() -> driver.getRightTriggerAxis() > 0.5).whileTrue(intake.runIntake());
            new JoystickButton(driver, XboxController.Button.kRightBumper.value).onTrue(intake.clearGamePiece());
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
