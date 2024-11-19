# Simulating Swerve Drivetrain

> ⚠️ **Note**
>
> You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

> 💡 **Tip**
> 
> This realistic simulation allows you to accomplish a variety of tasks without the real robot. You can practice driving, test autonomous routines, and fine-tune advanced functions such as auto-alignment, just to name a few.



## Creating the Configuration for a Swerve Drive Simulation

The `DriveTrainSimulationConfig` object represents the contains the physical properties of a swerve drivetrain.
You can store a `DriveTrainSimulationConfig` in your constant file or create it in the subsystem code, like this:

```java
// Create and configures a drivetrain simulation configuration
final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // specify gyro type (for realistic gyro drifiting and error simulation)
        .withGyro(GyroSimulation.getPigeon2())
        // specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(SwerveModuleSimulation.getMark4(
                DCMotor.getKrakenX60(1), // drive motor is a Kraken x60
                DCMotor.getFalcon500(1), // steer motor is a Falcon 500
                Amps.of(60), // The stator current limit for the drive motor is 60A
                SwerveModuleSimulation.WHEEL_GRIP.RUBBER_WHEEL.cof, // use the COF for rubber wheels
                3 // l3 gear ratio
        ))
        // configures the track length of track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // configures the bumper size
        .withBumperSize(Inches.of(30), Inches.of(30));

```

## Instantiate and Register a Swerve Drive Simulation

The `SwerveDriveSimulation` class represents a simulated swerve drivetrain on field.  It contains neccessary simulation code for motors and encoders to mimic the dynamics and controls of a real robot.  It also has collision space in the physics engine, so that it can interact with the field.

You can create the swerve drive like this:

```java
/* Create a swerve drive simulation */
this.swerveDriveSimulation = new SwerveDriveSimulation(
        driveTrainSimulationConfig, 
        new Pose2d(3, 3, new Rotation2d())
);
```

The simulation needs to be added to a simulation world to work correctly: 

```java
// Register the drivetrain simulation
SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation); 
```

## Manipulating the Simulated Swerve

The `SwerveDriveSimulation` only contains only neccessary code to model the physics behavior of a swerve drive, it does not contain code to drive it. 

Users need to manipulate the simulated swerve drive to make the simulation work.  There are two ways to do it:

<details>
    <summary><strong>Option1: The easy way</strong></summary>
    <p>See <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/3.1_SWERVE_SIM_EZ_MODE.html">Swerve Simulation: Simplified Swerve Simulation</a> >></p>
</details>

<details>
    <summary><strong>Option2: The recommended solution</strong></summary>
    <p>See <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/3.2_SWERVE_SIM_HARDWARE_ABSTRACTION.html">Swerve Simulation: Hardware Abstractions</a> >></p>
</details>

<div style="display:flex">
    <h3 style="width:49%"><< Prev: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/2_USING_THE_SIMULATED_ARENA.html">Using the Simulated Arena Object</a></h3>
</div>