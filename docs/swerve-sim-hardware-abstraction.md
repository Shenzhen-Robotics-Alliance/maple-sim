# Swerve Simulation: Hardware Abstractions

*This approach to simulating swerve drive accurately mimics the behavior of your drivetrain code by running the exact same code used on the real robot directly on the simulated robot. While this ensures high fidelity in the simulation, it does require a significant amount of effort to set up properly.*

**For a more comprehensive understanding of how this works, please check out the [AdvancedSwerveDrive-maple-sim example](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/).**

!!! warning
      You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

---
## 0. IO Abstraction

!!! tip 
      If you're using AdvantageKit, your code is already IO-abstracted. You do not need to restructure your code to use maple-sim.

The core idea is that the code for your subsystem interacts with an IO interface that can have multiple implementations.

The code runs EXACTLY the same regardless of which IO interface it interacts with, whether on a real robot or in a physics engine.

![](media/hardware%20abstraction.svg)

Optionally, you can use the [log-replay technology](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/what-is-advantagekit/index.md) with [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit).

To implement IO abstraction, you need to organize the code for EACH subsystem according to this structure:

```
subsystems/
├── MySubsystem/
│   ├── MySubsystem.java
│   ├── MySubsystemIO.java
│   ├── MySubsystemIOTalonFX.java
│   ├── MySubsystemIOSparkMax.java
│   └── MySubsystemIOSim.java
```

- `MySubsystem.java`:  The main code that controls the subsystem.
- `MySubsystemIO.java`: Defines the IO interface, specifying the abstract inputs and outputs of the subsystem.
- `MySubsystemIOTalonFX.java`: A hardware implementation of the IO interface using Falcon motors.
- `MySubsystemIOSparkMax.java`: A hardware implementation of the IO interface using Neo motors.
- `MySubsystemIOSim.java`: he simulation implementation of the IO interface, which runs the physics simulation.

Reference: [io-interface.md from Advantage Kit Documents](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/recording-inputs/io-interfaces.md)

---
## 1. Interacting with the Gyro through IO abstraction
The simulated gyro includes measurement errors and will drift if the robot collides with an obstacle. For accurate odometry simulation, use the simulated gyro to update the odometry.

Create `GyroIOSim.java`, which implements `GyroIO` by wrapping around the methods of `GyroSimulation`:

### IO interface
```java
// Example gyro interface
public interface GyroIO {
    Rotation2d getGyroRotation();
    AngularVelocity getGyroAngularVelocity();
}
```

### Real Implementation

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
    public AngularVelocity getGyroAngularVelocity() {
        return pigeon2.getAngularVelocity();
    }
}
```

### Simulation Implementation

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
    public AngularVelocity getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocity();
    }
}
```

If you're using AdvantageKit, you'll want to update the `GyroInputs` by retrieving data from the `GyroSimulation`.

An example of implementing the Gyro interface in an AdvantageKit environment can be found [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/drive/GyroIOSim.java).

---
## 2. Interacting with the Simulated Modules through IO abtraction

Similar to the gyro, you also need to create hardware abstractions for the swerve modules.

To implement `ModuleIO` using the simulator, we need to obtain the reference to the simulated motor controller of the motors, and use the following API references in `ModuleIOSim`:

```java
// This is only an example simulation IO implement, please change the code according to your ModuleIO interface
public class ModuleIOSim implements ModuleIO {
    // reference to module simulation
    private final SwerveModuleSimulation moduleSimulation;
    // reference to the simulated drive motor
    private final SimulatedMotorController.GenericMotorController driveMotor;
    // reference to the simulated turn motor
    private final SimulatedMotorController.GenericMotorController turnMotor;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures a generic motor controller for drive motor
        // set a current limit of 60 amps
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(60));
        this.turnMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(20));
    }
    
    @Override // specified by ModuleIO interface
    public void setDriveOutputVoltage(Voltage voltage) {
        this.driveMotor.requestVoltage(voltage);
    }

    @Override // specified by ModuleIO interface
    public void setSteerOutputVoltage(Voltage voltage) {
        this.turnMotor.requestVoltage(voltage);
    }
    
    @Override // specified by ModuleIO interface
    public Rotation2d getSteerFacing() {
        return this.moduleSimulation.getSteerAbsoluteFacing();
    }
    
    @Override // specified by ModuleIO interface
    public Angle getSteerRelativePosition() {
        return moduleSimulation.getSteerRelativeEncoderPosition().divide(moduleSimulation.STEER_GEAR_RATIO));
    }
    
    @Override // specified by ModuleIO interface
    public Angle getDriveWheelrPositiond() {
        return moduleSimulation.getDrieWheelFinalPosition()
    }
}
```

---
## 3. Instantiating Simulation IO Implementations

When running the simulator, you can instantiate the above simulation IO implementations to allow the robot to run within the simulation environment.

```java
// creation the swerve simulation (please refer to previous documents)
this.swerveDriveSimulation = new SwerveDriveSimulation(...);

// 

this.drive = new Drive(
        new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation()),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[0]),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[1]),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[2]),
        new ModuleIOSim(this.swerveDriveSimulation.getModules()[3])
);
```


