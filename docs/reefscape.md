# 2025 Reefscape Simulation
![alt text](media/fd_frc_socialgraphics_fb_post.png)

## ***CORAL*** and ***ALGAE*** on the Field
***CORAL*** and ***ALGAE*** can be added to the field as game pieces:

```java
SimulatedArena.getInstance().addGamePiece(new ReefscapeCoral(
    // We must specify a heading since the coral is a tube
    new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));
```

You can visualize them by calling:

```java
Logger.recordOutput("FieldSimulation/Algae", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
Logger.recordOutput("FieldSimulation/Coral", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
```

And display the data in AdvantageScope:

![](./media/reefscape%20coral%20algae%20ascope.png)

![Reefscape Game Pieces](./media/reefscape%20game%20pieces.gif)

!!! abstract "Detailed Documents on Game Pieces Simulation"
    **[:octicons-arrow-right-24: Adding Game Pieces to the Field](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/#3-adding-game-pieces-to-the-field)**
    
    **[:octicons-arrow-right-24: Visualizing Game Pieces](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/#4-visualizing-game-pieces)**


## The ***CORAL-ALGAE*** Stack
***CORAL*** are staged on the field with ***ALGAE*** on top. You can place these stacks on the field:

```java
SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2,2)));
```

Or reset the layout by performing a field reset:

```java
SimulatedArena.getInstance().resetFieldForAuto();
```

!!! tip
    When you display ***CORAL*** and ***ALGAE*** using the method specified above, the ***CORAL*** and ***ALGAE*** in the stacks will also be displayed.
![alt text](./media/reefscape%20stack%20ascope.png)


!!! tip
    Stacks will collapse and become a ***CORAL*** and ***ALGAE*** on the field if you hit them:

![](./media/reefscape%20stack.gif)

## Interacting with ***CORAL*** and ***ALGAE***
Users can use `IntakeSimulation` to simulate the interaction between robot intakes and the game pieces. 

```java
this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Coral",
        // Specify the drivetrain to which this intake is attached
        driveTrainSimulation,
        // Width of the intake
        Meters.of(0.4),
        // The extension length of the intake beyond the robot's frame (when activated)
        Meters.of(0.2),
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        1);
```

!!! abstract "Detailed Documents on IntakeSimulation"
    **[:octicons-arrow-right-24: Simulating Intake](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/)**

!!! tip
    - If your `IntakeSimulation` is targeted to ***CORAL***, it will also be able to grab the ***CORAL*** from the ***CORAL-ALGAE*** stack. And the ***ALGAE*** will fall to the ground as the ***CORAL*** disapear.
    - Vise versa, if your `IntakeSimulation` is targeted to ***ALGAE***, it will also be able to grab the ***ALGAE*** from the ***CORAL-ALGAE*** stack. And the ***CORAL*** will fall to the ground as the ***ALGAE*** disapear.


## Launching ***ALGAE*** into the air
***ALGAE*** can be launched into the air, and the simulation will detect if it reaches its target—the ***NET***.

```java
ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));
SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        new Translation2d(),
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        0.4, // initial height of the ball, in meters
        9, // initial velocity, in m/s
        Math.toRadians(70)) // shooter angle
        .withProjectileTrajectoryDisplayCallBack(
            (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
            (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
```

See [Simulating Projectiles](./simulating-projectiles.md).

![](./media/launching%20algae.gif)
