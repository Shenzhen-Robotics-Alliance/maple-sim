# Simulating Swerve Drivetrain

> ⚠️ **Note**
>
> You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

> 💡 **Tip**
> 
> This realistic simulation allows you to accomplish a variety of tasks without the real robot. You can practice driving, test autonomous routines, and fine-tune advanced functions such as auto-alignment, just to name a few.



## Creating a Swerve Drive Simulation

The `SwerveDriveSimulation` object represents the simulation for a swerve drivetrain. To create it, you need to provide some details about the drivetrain:

```java
// Create and configures a drivetrain simulation configuration
final DriveTrainSimulationConfig driveTrainSimulationConfig =
    DriveTrainSimulationConfig.Default()
        .withGyro(GyroSimulation.getPigeon2())
        .withSwerveModule(SwerveModuleSimulation.getMark4(
            DCMotor.getKrakenX60(1), // drive motor is a Kraken x60
            DCMotor.getFalcon500(1), // steer motor is a Falcon 500
            Amps.of(60), // current limit: 60 Amps
            SwerveModuleSimulation.WHEEL_GRIP.RUBBER_WHEEL.cof, // use COF of rubber wheels
            3 // l3 gear ratio
        ))
        .withCustomModuleTranslations(Drive.getModuleTranslations())
        .withBumperSize(Inches.of(30), Inches.of(30));

/* Create a swerve drive simulation */
this.swerveDriveSimulation = new SwerveDriveSimulation(
    driveTrainSimulationConfig, 
    new Pose2d(3, 3, new Rotation2d())
);

// Register the drivetrain simulation
SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation); 
```

## Manipulating the Simulated Swerve

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