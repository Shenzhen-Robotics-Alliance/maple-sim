# Simulating Intake
> ⚠️ **Note**
>
> You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

## Overview
Intakes, or mechanisms that extend from the robot to collect game pieces from the field, are simulated by the `IntakeSimulation` class.

The intake is modeled as a 2D shape attached to one side of the robot's chassis. Intakes are represented as rectangles with a fixed width.

<div style="display: flex;">
    <img src="media/intakesim2.png" style="width: 49%;">
    <img src="media/intakesim3.png" style="width: 49%;">
</div>

When deactivated, the rectangle is removed from the collision space, representing the intake retracting back into the robot’s frame.

When activated, the rectangle is added to the robot's collision space in the physics engine, simulating an intake that extends outward for a certain length.

This class simulates an idealized "touch it, get it" intake, meaning that when the intake is turned on, game pieces of the specified type are immediately collected upon contact — regardless of whether they are pushed towards the intake.

> 💡 **Tip**
>
> This simulation is intended for testing code and does not exactly replicate how your real intake mechanism works.


## Creating Intake Simulation
`IntakeSimulation` is an abstract class. You need to create a subclass that inherits from it to represent the intake simulation. Let's call it `IntakeIOSim`.

```java
// subsystems/intake/IntakeIOSim.java
public class IntakeIOSim extends IntakeSimulation {
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        super(
                "Note", // This intake can collect game pieces of type "Note"
                driveTrain, // Specify the drivetrain to which this intake is attached
                0.6, // Width of the intake
                0.1, // The extension length of the intake beyond the robot's frame
                // For under-bumper intakes, set this to 0.01
                IntakeSide.BACK, // The intake is mounted on the back side of the chassis
                1 // The intake can hold up to 1 note
        );
    }
}
```

## Using intake simulation
Next, implement the methods defined by the `IntakeIO` interface.

Intakes can be turned on and off by calling `startIntake()` and `stopIntake()`.
Most intakes detect game pieces inside the mechanism (usually with a beam breaker sensor). You can simulate this by checking `gamePiecesInIntakeCount`.

```java
public class IntakeIOSim extends IntakeSimulation implements IntakeIO {
    ...

    @Override // Defined by IntakeIO
    public void setRunning(boolean runIntake) {
        if (runIntake)
            super.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        else
            super.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    @Override // Defined by IntakeIO
    public boolean isNoteInsideIntake() {
        return this.gamePiecesInIntakeCount != 0; // True if there is a game piece in the intake
    }

    @Override // Defined by IntakeIO
    public void launchNote() {
        this.gamePiecesInIntakeCount = 0; // Clears the game piece from the intake
    }
}

```

## Combining Intake/Feeder Simulation with Shooter Simulation
Some intakes are directly connected to shooters, while others feed into a mechanism such as a feeder.

If you take the indefinite integral of the intake/feeder voltage over time since the note entered the intake, it gives an approximation of the note's position.
When the integral reaches a certain threshold, you can notify the `FlyWheelIOSim` to shoot the note out. See [Simulating GamePiece Projectiles](./6_SIMULATING_PROJECTILES.MD) for more details.

An example of simulating an intake together with flywheels can be found [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java).


<div style="display:flex; justify-content: space-between; width: 100%">
    <h3><< Prev: <a href="./4_SIMULATING_SWERVE_DRIVETRAIN.md">Using the Simulated Arena Object</a></h3>
    <h3>Next: <a href="./6_SIMULATING_PROJECTILES.MD">Simulating GamePiece Projectiles</a> >></h3>
</div>