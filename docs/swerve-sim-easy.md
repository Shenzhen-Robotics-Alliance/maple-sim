# Swerve Simulation: Simplified Swerve Simulation

*This approach emphasizes ease of use while maintaining a reasonably accurate model of robot behavior. Although the physics simulation is realistic enough to accurately mimic your drivetrain, the code used to manipulate the simulated drivetrain is embedded into maple-sim for convenience. As a result, it may differ slightly from the code running on your real robot.*

**This document is based on the [BaseTalonSwerve-maple-sim Example](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/dev/templates/BaseTalonSwerve-maple-sim).  Check the project for a more detailed understanding.**

!!! warning
      You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

---
## 0. Abstracting your drive subsystem
Before we start, we need to create a `SwerveDrive` interface or abstract class that specifies the functions of the drive subsystem.

![](./media/subsystem%20abstraction.svg)

#### Example drive subsystem abstraction: 
[View original source](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/dev/templates/BaseTalonSwerve-maple-sim/src/main/java/frc/robot/subsystems/drive/SwerveDrive.java)
```java
package frc.robot.subsystems.drive;

/**
 * Represents a subsystem responsible for controlling the drive system of a robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position and orientation, and other related tasks.
 */
public interface SwerveDrive extends Subsystem {
    void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getMeasuredSpeeds();

    Rotation2d getGyroYaw();

    Pose2d getPose();

    void setPose(Pose2d pose);

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);
    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
}
```

**Next, we implement the interface using the real hardware of the robot. [View Example](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/dev/templates/BaseTalonSwerve-maple-sim/src/main/java/frc/robot/subsystems/drive/TalonSwerve.java)**

---
## 1. Creating a simulation drive subsystem implementation

Now, let's create an implementation of the `SwerveDrive` interface using a simulated swerve drivetrain.

To do this, we will use the `SimplifiedSwerveDriveSimulation` class. This class manages and controls a [SwerveDriveSimulation](https://shenzhen-robotics-alliance.github.io/maple-sim/3_SWERVE_SIMULATION_OVERVIEW.html#creating-a-swerve-drive-simulation), similar to how your user code controls the physical chassis. Based on the commands sent by the user, it runs closed loops on the virtual drive/steer motors in the simulated drivetrain, mimicking the behavior of the real swerve drive.

[View original source](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/dev/templates/BaseTalonSwerve-maple-sim/src/main/java/frc/robot/subsystems/drive/MapleSimSwerve.java)
```java
package frc.robot.subsystems.drive;

public class MapleSimSwerve implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    public MapleSimSwerve() {
        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);
    }
}
```

Now we can create wrapper methods to implement the APIs specified by the `SwerveDrive` interface:

```java
package frc.robot.subsystems.drive;

public class MapleSimSwerve implements SwerveDrive {
    ... // previous code not shown

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                new Translation2d(),
                fieldRelative,
                true);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
    }
}
```

---
## 2. Using the simulated drivetrain

In the `RobotContainer`, we store an instance of the `SwerveDrive` interface. Depending on the robot’s current mode, we instantiate different types of implementations.
```java
// in RobotContainer.java
private final SwerveDrive drive;

public RobotContainer() {
    if (Robot.isReal()) {
        this.drive = new TalonSwerve(); // Real implementation
    }
    else {
        this.drive = new MapleSimSwerve(); // Simulation implementation
    }
}
```

Our drive commands and auto builders should **ALWAYS** take the interface as a dependency, regardless of whether it’s a real implementation or a simulation.

[View original source](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/dev/templates/BaseTalonSwerve-maple-sim/src/main/java/frc/robot/commands/TeleopSwerve.java)
```java
package frc.robot.commands;

public class JoystickDrive extends Command {
    private SwerveDrive drive;
    ... // Other requirements

    public JoystickDrive(
            SwerveDrive drive, // Take an instance of the SwerveDrive interface as a dependency
            ... // Other requirements (e.g., driver gamepad axis suppliers)
    ) {
        this.drive = drive;
        addRequirements(drive);
        ... // Process other requirements
    }

    @Override
    public void execute() {
        ... // Process command logic

        drive.drive(...); // Execute the logic through APIs specified by the SwerveDrive interface
    }
}

```

We can also configure the auto builder directly within the `SwerveDrive` interface, enabling autonomous driving in both the real world and simulation.

```java
public interface SwerveDrive extends Subsystem {
    ... // previous code not shown

    /** Configures the PathPlanner auto builder */
    default void configurePPAutoBuilder() {
        AutoBuilder.configure(
                // Use APIs from SwerveDrive interface
                this::getPose, 
                this::setPose,
                this::getMeasuredSpeeds,
                (speeds) -> this.drive(speeds, false, true),
                
                // Configure the Auto PIDs
                new PPHolonomicDriveController(
                    Constants.AUTO_TRANSLATIONAL_PID_CONSTANT, 
                    Constants.AUTO_ROTATIONAL_PID_CONSTANT),
                
                // Specify the PathPlanner Robot Config
                Constants.ROBOT_CONFIG,
                
                // Path Flipping: Determines if the path should be flipped based on the robot's alliance color
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red),
                
                // Specify the drive subsystem as a requirement of the command
                this);
    }
}

```
