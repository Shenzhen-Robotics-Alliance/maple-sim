<p align="center">
  <img src="./docs/media/team_logo.png" width="20%"  alt="team logo"/>
  <img src="./docs/media/icon.png" width="79%"  alt="project logo"/>
</p>

### A Way to Implement Physics Engine in FRC Java Robot Simulation

## Why physics engine?
Simulation engine is a software that provides approximate simulation of certain physical systems.  In maple-sim, we used an open source java rigid body dynamics engine [dyn4j](), which is capable of simulating the forces on 2d and collisions between 2d rigid shapes.

![physics engine illustration](docs/media/physics%20engine.png)

Prior to maple-sim, robot simulations focus on the robot itself.

But through the implementation of the physics engine, it is now possible to simulate the interaction between the robot and the surrounding environment.  Robots can now interact with the field and the game pieces in your simulation.

Having such realistic robot simulation means a lot of things, you will be able to test almost every aspect of your robot's function like autonomous modes, auto alignments, shooter optimization and a lot more things **without having a real robot**.

Ultimately, your simulation can achieve a level that it is realistic enough to be played like a "video game".

<iframe width="80%" src="https://www.youtube.com/embed/5jr1L8xWpog"></iframe>

## Simulation Details

### The Field has Collision Space
The standout feature of this project is the integration of the [dyn4j physics engine](https://github.com/dyn4j/dyn4j), which allows for the creation of a highly realistic and interactive simulation environment.

In our simulator, the robot can collide with field obstacles, interact with opposing robots, and engage with game pieces, providing a dynamic and immersive experience.

![robot_physics_simulation.gif](docs/media/robot_physics_simulation.gif)

### Swerve-Drive Physics Simulation
Previous swerve drive simulations modeled the drivetrain using several [DCMotorSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/DCMotorSim.html) instances. In contrast, this project's drivetrain simulation calculates the frictional and propelling forces on each swerve module. These forces are then used to model the drivetrain as a rigid body within the physics engine.

[[Source Code]() | [Guide]() | [Example]()]
![swervedrivesim.gif](docs/media/swervedrivesim.gif)

### Game Pieces and Intake Simulation

In our simulation, game pieces on the field have collision boundaries and can interact with the robot. The simulator also supports a fixed intake module on the robot, enabling it to automatically grab game pieces upon contact.
[[Source Code]() | [Guide]() | [Example]()]

![intakesim.gif](docs/media/intakesim.gif)

### Opponent Robots Simulation

Simulated opponent robots can be either manually controlled using a gamepad for defensive play or set to follow pre-programmed cycle paths. Like real robots, these opponents have collision boundaries, allowing drivers to practice both defensive and offensive strategies effectively.

[[Guide]() | [Example]()]

![opponentrobotsim.gif](docs/media/opponentrobotsim.gif)

### Odometry Simulation

By modeling the interaction between the wheels and the floor, our simulation can replicate odometry measurement errors caused by skidding.

Additionally, the IMU simulation incorporates drifting.  Hard impacts on the virtual robot will cause the IMU to drift, just like in real-world.

[[Guide]() | [Example]()]
![Odometry Sim](docs/media/odometrysim.gif)

## Quick Start

Please see the following guides: 

1. [Hardware Abstractions](./docs/1_HARDWARE_ABSTRACTIONS.md)
2. [Installing maple-sim](./docs/2_INSTALLING_MAPLE_SIM.md)
3. [Using the Simulated Arena](./docs/3_USING_THE_SIMULATED_ARENA.md)
4. [Simulating Swerve Drivetrain](./docs/4_SIMULATING_SWERVE_DRIVETRAIN.md)
5. [Simulating Intake](./docs/5_SIMULATING_INTAKE.md)
6. [Simulating Opponent Robots](./docs/6_SIMULATING_OPPONENT_ROBOTS.md)

## Examples and Templates

1. [Advanced Swerve Drive Project](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main/templates/AdvantageKit_AdvancedSwerveDriveProject): 6328's swerve drive project utilizing AdvantageKit, modified to have advanced drivetrain simulations.  The changelog from the original project is [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/1667aa80170e6733d8eaee866da0297e762402fa). 
2. [Maple-Swerve-Skeleton](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton): Our custom swerve drive project based on Advanced Swerve Drive Project with drivetrain simulation, vision simulation and some convenient controlling features.
3. [5516-2024-OffSeason-RobotCode](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/tree/main/example/5516-2024-OffSeason): Our 2024 off-season robot code that implements a variety of advanced simulations.  It can run on real robot and be played like a video game.  [Videos](https://www.youtube.com/watch?v=5jr1L8xWpog&list=PLFS6A0KifAK1ycwlzIlvvFJkWNsQHVjSN)