# Simulating Swerve Drivetrain

> ⚠️ **Note**
>
> You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

> 💡 **Tip**
> 
> This realistic simulation allows you to accomplish a variety of tasks without the real robot. You can practice driving, test autonomous routines, and fine-tune advanced functions such as auto-alignment, just to name a few.



## Creating the Configuration for a Swerve Drive Simulation

The `DriveTrainSimulationConfig` object encapsulates the physical properties and configuration of a swerve drivetrain within the simulation environment. It allows you to specify key components such as motor types, gyro configuration, swerve module dynamics, and robot geometry. 

```java
// Create and configure a drivetrain simulation configuration
final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(GyroSimulation.getPigeon2())
        
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(SwerveModuleSimulation.getMark4(
                DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                Amps.of(60), // The stator current limit for the drive motor is 60A
                SwerveModuleSimulation.WHEEL_GRIP.COLSONS.cof, // Use the COF for Colson Wheels
                3 // Gear ratio (l3 gear ratio)
        ))
        
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));
```

## Instantiate and Register a Swerve Drive Simulation

The `SwerveDriveSimulation` class represents a simulated swerve drivetrain within the simulation environment. It provides the necessary code to simulate motors, encoders, and the dynamic behavior of a real robot, including accurate handling of controls, motor response, and motion. Additionally, it interacts with the field through its collision space, allowing the simulation to react with other objects in the physics engine.

You can instantiate the swerve drive simulation using the following code:

```java
/* Create a swerve drive simulation */
this.swerveDriveSimulation = new SwerveDriveSimulation(
        // Specify Configuration
        driveTrainSimulationConfig, 
        // Specify starting pose
        new Pose2d(3, 3, new Rotation2d())
);
```

The simulation must be registered to the simulation world for it to function correctly:

```java
// Register the drivetrain simulation to the default simulation world
SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation); 
```

## Manipulating the Simulated Swerve
<details>
    <summary><strong>Option 1: The simple approach</strong></summary>
    <p><em>This approach emphasizes ease of use while maintaining a reasonably accurate model of robot behavior. Although the physics simulation is realistic enough to accurately mimic your drivetrain, the code used to manipulate the simulated drivetrain is embedded into maple-sim for convenience. As a result, it may differ slightly from the code running on your real robot.</em></p>
    <h4>See <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/3.1_SWERVE_SIM_EZ_MODE.html">Swerve Simulation: SimplifiedSwerveSimulation</a></h4>
</details>
<details>
    <summary><strong>Option 2: The professional approach</strong></summary>
    <p><em>This approach to simulating swerve drive accurately mimics the behavior of your drivetrain code by running the exact same code used on the real robot directly on the simulated robot. While this ensures high fidelity in the simulation, it does require a significant amount of effort to set up properly.</em></p>
    <h4>See <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/3.2_SWERVE_SIM_HARDWARE_ABSTRACTION.html">Swerve Simulation: Hardware Abstraction</a></h4>
</details>
<details>
    <summary><strong>Option 3: Simulation with CTRE devices</strong></summary>
    <p>Comming soon, <a href='https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/CTRE-simulation-support'>view progess</a></p>
</details>

<div style="display:flex">
    <h3 style="width:49%"><< Prev: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/2_USING_THE_SIMULATED_ARENA.html">Using the Simulated Arena Object</a></h3>
    <h3 style="width:49%" align="right">Next: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/4_SIMULATING_INTAKE.html">Simulating Intake</a> >></h3>
</div>