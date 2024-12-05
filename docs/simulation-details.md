# Simulation Details

## Swerve-Drive Simulation Dynamics with Interactive Field
The standout feature of this project is the integration of the [dyn4j physics engine](https://github.com/dyn4j/dyn4j), which enables a highly realistic and interactive simulation environment.

In traditional swerve drive simulations, the drivetrain is typically modeled using several [DCMotorSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/DCMotorSim.html) instances to simulate motor behavior. 

In maple-sim, the drivetrain simulation goes a step further by calculating the frictional and propelling forces on each swerve module. 
These forces are used to model the drivetrain as a rigid body, allowing for more accurate interaction with the field environment.

![swerve drive dynamics.gif](media%2Fswerve%20drive%20dynamics.gif)

The realistic simulation dynamics allow you to test and refine auto paths as they would perform in real-world conditions.

![path following simulation.gif](media%2Fpath%20following%20simulation.gif)

**[View Full Document on Swerve Simulation](./swerve-simulation-overview.md) >>>**

## Odometry + Vision Simulation

By modeling the interaction between the wheels and the floor, our simulation can replicate odometry measurement errors caused by skidding.

Additionally, the IMU simulation includes drift, where hard impacts on the virtual robot will cause the IMU to behave similarly to how it would in real-world scenarios.

![Odometry Sim](media/odometry%20simulation.gif)

Beyond odometry, you can integrate [photonlib](https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html) to simulate how vision-based odometry can correct your robot’s position.

![vision simulation.gif](media%2Fvision%20simulation.gif)

## Game Pieces and Intake Simulation
In **maple-sim**, game pieces on the field have collision boundaries and can interact with the robot.
The simulator also supports a fixed intake module, allowing the robot to automatically collect game pieces upon contact.

![game pieces simulation.gif](media%2Fgame%20pieces%20simulation.gif)

![intakesim.gif](media/intakesim.gif)

**[View Full Document on Intake Simulation](./simulating-intake.md) >>>**

## Projectile Simulation
In FRC, game pieces are often launched into the air. 
**maple-sim** offers a straightforward physics simulation to model the behavior of these projectiles.

![projectile simulation.gif](media%2Fprojectile%20simulation.gif)

**[View Full Document on Projectile Simulation](./simulating-projectiles.md) >>>**

## Opponent Robots Simulation

Simulated opponent robots can be manually controlled with a gamepad for defensive play or set to follow pre-programmed cycle paths. 
Just like real robots, these opponents have collision boundaries, enabling drivers to practice both defensive and offensive strategies effectively.
![opponentrobotsim.gif](media/opponent%20robot%20simulation.gif)

