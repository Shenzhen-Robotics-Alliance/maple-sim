# 2026 Rebuilt Simulation
![alt text](media/frc_socialgraphics_fb_post_2026.jpeg)

## ***FUEL*** on the Field
!!! info ""
Fuel can be added to the field as game pieces:

```java
SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(2,2)));
```

You can visualize them by calling:

```java
Logger.recordOutput("FieldSimulation/Fuel", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
```

And display the data in AdvantageScope:

![](./media/reefscape%20coral%20algae%20ascope.png)

![Reefscape Game Pieces](./media/reefscape%20game%20pieces.gif)

!!! abstract "Detailed Documents on Game Pieces Simulation"
**[:octicons-arrow-right-24: Adding Game Pieces to the Field](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/#3-adding-game-pieces-to-the-field)**

    **[:octicons-arrow-right-24: Visualizing Game Pieces](https://shenzhen-robotics-alliance.github.io/maple-sim/using-the-simulated-arena/#4-visualizing-game-pieces)**

## Interacting with ***FUEL***
!!! info ""
Users can use `IntakeSimulation` to simulate the interaction between robot intakes and the game pieces.

```java
this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Fuel",
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
- If the game in involved multiple types of game pieces and your `IntakeSimulation` is targeted to only one, it will be only able to grab that type of game piece.

## Launching ***FUEL*** into the air
!!! info ""
***FUEL*** can be launched into the air, and the simulation will detect if it reaches its target—the ***HUB***.

```java
RebuiltFuelOnFly.setHitTargetCallBack(() -> System.out.println("FUEL hits HUB!"));
SimulatedArena.getInstance()
    .addGamePieceProjectile(new RebuiltFuelOnFly(
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