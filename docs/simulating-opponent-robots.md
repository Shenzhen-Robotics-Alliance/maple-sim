# Simulating Opponent Robots - an option

!!! warning
    You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

---

## 0. Overview

Opponent robots can be added to the field for realistic driver practice. They can be programmed to perform the following tasks:

- Automatically cycle across the field, helping drivers practice offense skills with other robots present.
- Automatically run feed-cycles to deliver feed-shot notes, assisting with front-field cleanup and feeder strategies.
- Be controlled by a joystick to play defense, allowing drivers to practice defense and counter-defense skills.

**This document discusses **one approach** to achieve opponent robot simulation based on the [AIRobotInSimulation class from Maple-Swerve-Skeleton](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/AIRobotInSimulation.java).  However, once you understand the principles, we encourage you to create your own simulation utility classes to fit your specific needs.**

---

## 1. Creating Opponent Robots

Opponent robots are simulated using the `SimplifiedSwerveDriveSimulation` class. A container class is required to manage the instances.

When opponent robots are not actively on the field, they are positioned in "queening" areas outside the field to avoid unnecessary interactions.

```java
public class AIRobotInSimulation extends SubsystemBase {
    /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };

    private final SimplifiedSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;

    public AIRobotInSimulation(int id) {
        this.id = id;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation = new SimplifiedSwerveDriveSimulation(new SwerveDriveSimulation(
            DRIVETRAIN_CONFIG, 
            queeningPose
        ));

        SimulatedArena.getInstance().addDriveTrainSimulation(
            driveSimulation.getDriveTrainSimulation()
        );
    }
}
```

---

## 2. Controlling Opponent Robots to Auto-Cycle

Use [PathPlanner](https://pathplanner.dev/home.html) to enable opponent robots to auto-cycle.

### Configuring PathPlanner

```java
public class AIRobotInSimulation extends SubsystemBase {
    ... // previous code not shown

    // PathPlanner configuration
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            55, // Robot mass in kg
            8,  // Robot MOI
            new ModuleConfig(
                    Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getFalcon500(1).withReduction(8.14), 60, 1), // Swerve module config
            0.6, 0.6 // Track length and width
    );

    // PathPlanner PID settings
    private final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

    /** Follow path command for opponent robots */
    private Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path, // Specify the path
                // Provide actual robot pose in simulation, bypassing odometry error
                driveSimulation::getActualPoseInSimulationWorld,
                // Provide actual robot speed in simulation, bypassing encoder measurement error
                driveSimulation::getActualSpeedsRobotRelative,
                // Chassis speeds output
                (speeds, feedforwards) -> 
                    driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
                driveController, // Specify PID controller
                PP_CONFIG,       // Specify robot configuration
                // Flip path based on alliance side
                () -> DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue)
                    .equals(DriverStation.Alliance.Red),
                this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
        );
    }
}
```

---

## 3. Controlling Opponent Robots with a Joystick

Write a joystick drive command to allow manual control of opponent robots for defense practice.

```java
public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
};

/** Joystick drive command for opponent robots */
private Command joystickDrive(XboxController joystick) {
    // Obtain chassis speeds from joystick input
    final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
            -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
            -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
            -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond));

    // Obtain driverstation facing for opponent driver station
    final Supplier<Rotation2d> opponentDriverStationFacing = () ->
            FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));

    return Commands.run(() -> {
            // Calculate field-centric speed from driverstation-centric speed
            final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    joystickSpeeds.get(),
                    FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                            .plus(Rotation2d.fromDegrees(180)));
            // Run the field-centric speed
            driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
            }, this)
            // Before the command starts, reset the robot to a position inside the field
            .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
}
```

---

## 4. Launching Gamepieces from Opponent Robots

Using the [Projectile Simulation in maple-sim](https://shenzhen-robotics-alliance.github.io/maple-sim/5_SIMULATING_PROJECTILES.html), opponent robots can deliver feed shots or score by launching gamepieces.

```java
private Command feedShot() {
    return Commands.runOnce(() -> SimulatedArena.getInstance()
            .addGamePieceProjectile(new NoteOnFly(
                            this.driveSimulation
                                    .getActualPoseInSimulationWorld()
                                    .getTranslation(),
                            new Translation2d(0.3, 0),
                            this.driveSimulation.getActualSpeedsFieldRelative(),
                            this.driveSimulation
                                    .getActualPoseInSimulationWorld()
                                    .getRotation(),
                            0.5,
                            10,
                            Math.toRadians(45))
                    .enableBecomeNoteOnFieldAfterTouchGround()));
}
```

---

## 5. Managing Opponent Robots

Opponent robots can be managed using a "behavior chooser," which is displayed on the dashboard. This allows for easy selection of behaviors such as disabling robots, running auto-cycles, or joystick driving.

```java
/** Build the behavior chooser of this opponent robot and send it to the dashboard */
public void buildBehaviorChooser(
        PathPlannerPath segment0,
        Command toRunAtEndOfSegment0,
        PathPlannerPath segment1,
        Command toRunAtEndOfSegment1,
        XboxController joystick) {
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    final Supplier<Command> disable =
            () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                    .andThen(Commands.runOnce(() -> driveSimulation.runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false)))
                    .ignoringDisable(true);

    // Option to disable the robot
    behaviorChooser.setDefaultOption("Disable", disable.get());

    // Option to auto-cycle the robot
    behaviorChooser.addOption(
            "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

    // Option to manually control the robot with a joystick
    behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

    // Schedule the command when another behavior is selected
    behaviorChooser.onChange((Command::schedule));

    // Schedule the selected command when teleop starts
    RobotModeTriggers.teleop()
            .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

    // Disable the robot when the user robot is disabled
    RobotModeTriggers.disabled().onTrue(disable.get());

    SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
}

/** Get the command to auto-cycle the robot relatively */
private Command getAutoCycleCommand(
        PathPlannerPath segment0,
        Command toRunAtEndOfSegment0,
        PathPlannerPath segment1,
        Command toRunAtEndOfSegment1) {
    final SequentialCommandGroup cycle = new SequentialCommandGroup();
    final Pose2d startingPose = new Pose2d(
            segment0.getStartingDifferentialPose().getTranslation(),
            segment0.getIdealStartingState().rotation());

    cycle.addCommands(
            opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
    cycle.addCommands(
            opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

    return cycle.repeatedly()
            .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
}
```

---

## 6. Initializing Opponent Robots in Simulation

Keep opponent robot instances in a static variable and initialize them during simulation startup.

```java
public static final AIRobotInSimulation[] instances = new AIRobotInSimulation[...]; // you can create as many opponent robots as you needs
public static void startOpponentRobotSimulations() {
    try {
        instances[0] = new AIRobotInSimulation(0);
        instances[0].buildBehaviorChooser(
                PathPlannerPath.fromPathFile("opponent robot cycle path 0"),
                Commands.none(),
                PathPlannerPath.fromPathFile("opponent robot cycle path 0 backwards"),
                Commands.none(),
                new XboxController(2));

        instances[1] = ...;
        instances[1].buildBehaviorChooser(
                PathPlannerPath.fromPathFile("opponent robot cycle path 1"),
                instances[1].shootAtSpeaker(),
                PathPlannerPath.fromPathFile("opponent robot cycle path 1 backwards"),
                Commands.none(),
                new XboxController(3));

        ... // create more opponent robots if you need
    } catch (Exception e) {
        DriverStation.reportError("Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
    }
}
```

Call this initialization in the simulation lifecycle:

```java
// Robot.java
@Override
public void simulationInit() {
    AIRobotInSimulation.startOpponentRobotSimulations();
}
```
