# 2025 Reefscape Simulation
![alt text](media/fd_frc_socialgraphics_fb_post.png)

## CORAL and ALGAE on the Field
CORAL and ALGAE can be added to the field as game pieces:

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

For more details, see:
- [Adding Game Pieces to the Field](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/#3-adding-game-pieces-to-the-field)
- [Visualizing Game Pieces](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/#4-visualizing-game-pieces)

![Reefscape Game Pieces](./media/reefscape%20game%20pieces.gif)

## Launching ALGAE
ALGAE can be launched into the air, and the simulation will detect if it reaches its target—the NET.

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

## The CORAL-ALGAE Stack
CORAL are staged on the field with ALGAE on top. You can place these stacks on the field:

```java
SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2,2)));
```

Or reset the layout by performing a field reset:

```java
SimulatedArena.getInstance().resetFieldForAuto();
```

To visualize the stacks, use:

```java
Logger.recordOutput("FieldSimulation/StackedAlgae", 
    ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
Logger.recordOutput("FieldSimulation/StackedCoral", 
    ReefscapeCoralAlgaeStack.getStackedCoralPoses());
```

And display the data in AdvantageScope:

![](./media/reefscape%20stack%20ascope.png)

Stacks will collapse and become a CORAL and ALGAE on the field if you hit them:

![](./media/reefscape%20stack.gif)
