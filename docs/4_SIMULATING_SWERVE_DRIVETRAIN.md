# **Simulating Swerve Drivetrain**

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


## Interacting with the Gyro
The simulated gyro includes measurement errors and will drift if the robot collides with an obstacle. For accurate odometry simulation, use the simulated gyro to update the odometry.

You first need a hardware abstraction for your gyros. See [Hardware Abstraction](./1_HARDWARE_ABSTRACTIONS.md).

Create `GyroIOSim.java`, which implements `GyroIO` by wrapping around the methods of `GyroSimulation`:

```java
// Example gyro interface
public interface GyroIO {
    Rotation2d getGyroRotation();
    double getGyroAngularVelocity();
}
```

```java
// Simulation implementation
public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }
    
    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }
    
    @Override // specified by GroIOSim interface
    public double getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocityRadPerSec();
    }
}
```

```java
// real implementation with Pigeon2
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon2;
    public GyroIOPigeon2(...) {
        // Implementation details
    }
    
    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return pigeon2.getYaw();
    }
    
    @Override // specified by GroIOSim interface
    public double getGyroAngularVelocity() {
        return pigeon2.getAngularVelocity();
    }
}
```

If you're using AdvantageKit, you'll want to update the `GyroInputs` by retrieving data from the `GyroSimulation`.

An example of implementing the Gyro interface in an AdvantageKit environment can be found [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/GyroIOSim.java).

## Interacting with the Simulated Modules

Similar to the gyro, you also need to create hardware abstractions for the swerve modules.

To implement `ModuleIO` using the simulator, use the following API references in `ModuleIOSim`:

```java
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
    }
    
    @Override // specified by ModuleIO interface
    public void setDriveOutputVoltage(double volts) {
        this.moduleSimulation.requestDriveVoltageOut(volts);
    }

    @Override // specified by ModuleIO interface
    public void setSteerOutputVoltage(double volts) {
        this.moduleSimulation.requestSteerVoltageOut(volts);
    }
    
    @Override // specified by ModuleIO interface
    public Rotation2d getSteerFacing() {
        return this.moduleSimulation.getSteerAbsoluteFacing();
    }
    
    @Override // specified by ModuleIO interface
    public double getSteerRelativePositionRad() {
        return this.moduleSimulation.getSteerRelativeEncoderPositionRad();
    }
    
    @Override // specified by ModuleIO interface
    public double getDriveEncoderPositionRad() {
        return this.moduleSimulation.getDriveEncoderUnGearedPositionRad();
    }
}
```

An example of interacting with module simulations and simulating high-frequency odometry can be found [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java).

## Instantiating Simulation IO Implementations

When running the simulator, you can instantiate the above simulation IO implementations to allow the robot to run within the simulation environment.

```java
this.gyroSimulation = new GyroSimulation(...);
this.swerveDriveSimulation = new SwerveDriveSimulation(...);

this.drive = new Drive(
        new GyroIOSim(this.gyroSimulation),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[0]),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[1]),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[2]),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[3])
);
```

<div style="display:flex; justify-content: space-between; width: 100%">
    <h3><< Prev: <a href="./3_USING_THE_SIMULATED_ARENA.md">Simulating Swerve Drivetrain</a></h3>
    <h3>Next: <a href="./6_SIMULATING_PROJECTILES.MD"></a> >></h3>
</div>