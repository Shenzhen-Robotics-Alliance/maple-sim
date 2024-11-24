# Using the Simulated Arena Object

!!! note
      You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

## The simulation world
maple-sim simulates the interaction between the robot and its surrounding environment.
For FRC, this environment represents the FRC Arena of the current year's game.

## The `SimulatedArena` instance
The simulation world, containing obstacles, game pieces, and robots, is stored in a `SimulatedArena` object. Most of the time, you'll only need one simulation world instance.

```java
// Obtains the default instance of the simulation world, which is a Crescendo Arena.
SimulatedArena.getInstance();
// Overrides the default simulation
SimulatedArena.overrideInstance(SimulatedArena newInstance); 
```

## Updating the `SimulatedArena` instance

You need to call the update function to keep the simulation world updated.

```java
// simulation period method in your Robot.java
@Override
public void simulationPeriod() {
    SimulatedArena.getInstance().simulationPeriodic();
}
```

> ❌ **Important**
> 
> DO NOT call `SimulatedArena.getInstance().simulationPeriodic()` when running on a **REAL** robot.
> This will consume the resources of your roboRIO.

For each period of your robot (or each call to `SimulatedArena.getInstance().simulationPeriodic()`), the simulator processes a few sub-ticks, advancing the time incrementally and recalculating the physics model. This method ensures higher accuracy and enables simulation of high-frequency odometry.

By default, the simulator runs 5 iterations per robot period, with the default robot period being 0.02 seconds. This allows you to simulate a typical 50Hz robot with 250Hz odometry.

You can adjust the simulator's timing if necessary:

```java
// Set the robot period to 1/100 second and configure 3 sub-ticks per period
// This simulates a 100Hz robot with 300Hz odometry.
SimulatedArena.overrideSimulationTimings(0.01, 3);
```

> ❌ **Caution**
> 
> DO NOT override the timing if you are using AdvantageKit, as it only supports 50Hz robots.

## Adding Game Pieces to the Field
Game pieces, such as Crescendo Notes, can be added to the field as rigid bodies with collision detection.
Your robot can interact with these pieces by bumping into them or grabbing them (see [Intake Simulation](./simulating-intake.md)).

To add or clear game pieces from the field:

```java
// Add a Crescendo note to the field
SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));

// Clear all game pieces from the field
SimulatedArena.getInstance().clearGamePieces();
```

!!! tip 
      In the Crescendo Simulation Arena, notes are automatically dropped from the human player station when no notes are near the source zone.
      In addition to notes on field, **maple-sim** also provides simulation for projectiles, see [Simulating GamePiece Projectiles](./simulating-projectiles.md)


## Visualizing game pieces

You can retrieve the positions of game pieces, including those on the ground and those launched into the air:

```java
// Get the positions of the notes (both on the field and in the air)
List<Pose3d> notesPoses = SimulatedArena.getInstance().getGamePiecesByType("Note");

// Publish to telemetry using AdvantageKit
Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);
```

