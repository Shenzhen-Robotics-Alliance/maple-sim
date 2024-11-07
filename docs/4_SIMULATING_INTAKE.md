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
You need to create an instance of `IntakeSimulation` in your `IntakeIOSim`.

```java
// subsystems/intake/IntakeIOSim.java
public class IntakeIOSim {
    private final IntakeSimulation intakeSimulation;
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        this.intakeSimulation = new IntakeSimulation(
                "Note", // This intake can collect game pieces of type "Note"
                driveTrain, // Specify the drivetrain to which this intake is attached
                0.6, // Width of the intake
                0.1, // The extension length of the intake beyond the robot's frame
                // For under-bumper intakes, set this to 0.01
                IntakeSimulation.IntakeSide.BACK, // The intake is mounted on the back side of the chassis
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
public class IntakeIOSim implements IntakeIO {
    ...

    @Override // Defined by IntakeIO
    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        else
            intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    @Override // Defined by IntakeIO
    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    @Override // Defined by IntakeIO
    public void launchNote() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake())
            ShooterIOSim.launchNote(); // notify the simulated flywheels to launch a note
    }
}

```

> 💡 **Tip**
> 
> As shown in the code above, you can notify `FlyWheelIOSim` (the simulated flywheel mechansim) to shoot the note out when the note is passed to the shooter from the intake. See [Simulating GamePiece Projectiles](./6_SIMULATING_PROJECTILES.MD) for more details.
>
>
> If you want to simulate how the note moves inside the intake/feeder, you can take the indefinite integral of the intake/feeder voltage over time since the note entered the intake.  This gives an approximation of the note's position in the feeder.
> 
> An example of simulating an intake together with flywheels can be found [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java).


<div style="display:flex">
    <h3 style="width:49%"><< Prev: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/3_SWERVE_SIMULATION_OVERVIEW.html">Simulating Swerve Drivetrain</a></h3>
    <h3 style="width:49%" align="right">Next: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/5_SIMULATING_PROJECTILES.html">Simulating GamePiece Projectiles</a> >></h3>
</div>