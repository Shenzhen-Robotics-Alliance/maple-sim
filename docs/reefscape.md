# 2025 Reefscape Simulation
![alt text](media/fd_frc_socialgraphics_fb_post.png)

## ***CORAL*** and ***ALGAE*** on the Field
!!! info ""
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
!!! info ""
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
!!! info ""
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
!!! info ""
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
## Scoring ***CORAL*** on ***REEF***

!!! info ""
    You can eject a ***CORAL*** from your scoring mechanism during simulation. If a ***CORAL*** ejected into the air is in contact with a ***BRANCH*** at the correct angle, it will be attached to that branch. Maple-Sim will automatically detect the contact and simulate the scoring process (requires v0.3.3 or above).

=== "Scoring on L3 branch"
    ```java
    SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.35, 0),
            // Obtain robot speed from drive simulation
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            // The height at which the coral is ejected
            Meters.of(1.28),
            // The initial speed of the coral
            MetersPerSecond.of(2),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-35)));
    ```

=== "Scoring on L4 branch"
    ```java
    SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.46, 0),
            // Obtain robot speed from drive simulation
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            // The height at which the coral is ejected
            Meters.of(2.1),
            // The initial speed of the coral
            MetersPerSecond.of(1),
            // The coral is ejected vertically downwards
            Degrees.of(-90)));
    ```

!!! info
    ***CORAL***s that miss the target will be dropped on the floor:

![](./media/scoring%20coral%20on%20field.gif)

!!! info
    ***CORAL***s that fall on the ***TROUGH*** will remain on the trough:

![](./media/scoring%20coral%20on%20trough.gif)

!!! info
    To score a ***CORAL*** on a ***BRANCH***, it must:

    - Be in contact with the ***BRANCH***.
    - Be aligned with the direction of the ***BRANCH***.
    - Be moving in the correct direction (i.e., if the coral is moving upwards, it cannot score).

![](./media/scoring%20coral%20on%20L3.gif)
![](./media/scoring%20coral%20on%20L4.gif)

To obtain the amount of corals scored on each branch, use:

```java
Optional<ReefscapeReefSimulation> reefSimulation = ReefscapeReefSimulation.getInstance();
int amountOfCoralsOnBranch_G_L4 = 0;
if (reefSimulation.isPresent()) {
    amountOfCoralsOnBranch_G_L4 = reefSimulation.get().getBranches()
        [6] // branch G
        [3]; // L4
}
```
See the [javadocs](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/org/ironmaple/simulation/seasonspecific/reefscape2025/ReefscapeReefSimulation.html) for more info.
