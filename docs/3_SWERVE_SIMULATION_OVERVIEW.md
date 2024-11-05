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

// Create a gyro simulation
final GyroSimulation gyroSimulation = GyroSimulation.createPigeon2();

this.swerveDriveSimulation = new SwerveDriveSimulation(
    45, // robot weight in kg
    0.65, // track width in meters
    0.65, // track length in meters 
    0.74, // bumper width in meters
    0.74, // bumper length in meters
    SwerveModuleSimulation.getMark4( // creates a mark4 module
        DCMotor.getKrakenX60(1), // drive motor is a Kraken x60
        DCMotor.getFalcon500(1), // steer motor is a Falcon 500
        80, // current limit is 80 Amps
        DRIVE_WHEEL_TYPE.RUBBER, // rubber wheels
        3 // l3 gear ratio
    ),
    gyroSimulation, // the gyro simulation
    new Pose2d(3, 3, new Rotation2d()) // initial starting pose on the field
);

// register the drivetrain simulation
SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation); 
```

## Manipulating the Simulated Swerve

<details>
    <summary><strong>Option1: The easy way</strong></summary>
</details>

<details>
    <summary><strong>Option2: The recommended solution</strong></summary>
</details>

<div style="display:flex">
    <h3 style="width:49%"><< Prev: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/2_USING_THE_SIMULATED_ARENA.html">Using the Simulated Arena Object</a></h3>
</div>