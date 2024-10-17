<p align="center">
  <img src="./docs/media/team_logo.png" width="20%"  alt="team logo"/>
  <img src="./docs/media/icon.png" width="79%"  alt="project logo"/>
</p>

### Elevating FRC Java Robot Simulations to the Next Level with Physics Engines

# Why a Physics Engine?
A simulation engine is a powerful tool that provides realistic approximations of physical systems. With **maple-sim**, we integrate the open-source Java rigid-body dynamics engine, [dyn4j](https://github.com/dyn4j/dyn4j), capable of simulating 2D forces and collisions between rigid shapes. This integration transforms the scope of robot simulations by enabling realistic interactions between robots, field elements, and game pieces.

![physics engine illustration](docs/media/physics%20engine.png)

Before **maple-sim**, most FRC robot simulations focused solely on the robot itself—its sensors, movements, and internal operations. 
Now, through the power of physics simulation, **maple-sim** allows your robot to engage directly with its environment. 
Imagine testing robot interactions with obstacles, field elements, and game pieces, all within the simulated world.
A simulation that is realistic enough to **feel like a video game.**

[![Demo Video 1](./docs/media/demo%20video%20cover.png)](https://www.youtube.com/watch?v=CBx1_Dosgec)


With this advanced level of simulation, the possibilities are endless. You can:

- Test autonomous modes with pinpoint accuracy.
- Fine-tune advanced TeleOp enhancement features like pathfinding-auto-alignment.
- Optimize shooters and other subsystems, all while gathering meaningful data from simulated physics.

**And the best part? You can achieve all of this without needing a real robot on hand.**

# Simulation Details

## Swerve-Drive Simulation Dynamics with Interactive Field
The standout feature of this project is the integration of the [dyn4j physics engine](https://github.com/dyn4j/dyn4j), which enables a highly realistic and interactive simulation environment.

In traditional swerve drive simulations, the drivetrain is typically modeled using several [DCMotorSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/DCMotorSim.html) instances to simulate motor behavior. 

In maple-sim, the drivetrain simulation goes a step further by calculating the frictional and propelling forces on each swerve module. 
These forces are used to model the drivetrain as a rigid body, allowing for more accurate interaction with the field environment.

![swerve drive dynamics.gif](docs%2Fmedia%2Fswerve%20drive%20dynamics.gif)

The realistic simulation dynamics allow you to test and refine auto paths as they would perform in real-world conditions.

![path following simulation.gif](docs%2Fmedia%2Fpath%20following%20simulation.gif)

<div align="right">
    <h2><a href="./docs/4_SIMULATING_SWERVE_DRIVETRAIN.md">View Full Document on Swerve Simulation</a> >>></h2>
</div>

## Odometry + Vision Simulation

By modeling the interaction between the wheels and the floor, our simulation can replicate odometry measurement errors caused by skidding.

Additionally, the IMU simulation includes drift, where hard impacts on the virtual robot will cause the IMU to behave similarly to how it would in real-world scenarios.

![Odometry Sim](docs/media/odometry%20simulation.gif)

Beyond odometry, you can integrate [photonlib](https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html) to simulate how vision-based odometry can correct your robot’s position.

![vision simulation.gif](docs%2Fmedia%2Fvision%20simulation.gif)

<div align="right">
    <h2><a href="./docs/8_SIMULATING_VISION.md">View Full Document on Vision Simulation</a> >>></h2>
</div>

## Game Pieces and Intake Simulation
In **maple-sim**, game pieces on the field have collision boundaries and can interact with the robot.
The simulator also supports a fixed intake module, allowing the robot to automatically collect game pieces upon contact.

![game pieces simulation.gif](docs%2Fmedia%2Fgame%20pieces%20simulation.gif)

![intakesim.gif](docs/media/intakesim.gif)

<div align="right">
    <h2><a href="./docs/5_SIMULATING_INTAKE.md">View Full Document on Intake Simulation</a> >>></h2>
</div>

## Projectile Simulation
In FRC, game pieces are often launched into the air. 
**maple-sim** offers a straightforward physics simulation to model the behavior of these projectiles.

![projectile simulation.gif](docs%2Fmedia%2Fprojectile%20simulation.gif)

<div align="right">
    <h2><a href="./docs/6_SIMULATING_PROJECTILES.MD">View Full Document on Projectile Simulation</a> >>></h2>
</div>

## Opponent Robots Simulation

Simulated opponent robots can be manually controlled with a gamepad for defensive play or set to follow pre-programmed cycle paths. 
Just like real robots, these opponents have collision boundaries, enabling drivers to practice both defensive and offensive strategies effectively.
![opponentrobotsim.gif](docs/media/opponent%20robot%20simulation.gif)

<div align="right">
    <h2><a href="./docs/7_SIMULATING_OPPONENT_ROBOTS.md">View Full Document on Opponent Robots Simulation</a> >>></h2>
</div>

# Quick Start

Please see the following guides: 

1. [Hardware Abstractions](./docs/1_HARDWARE_ABSTRACTIONS.md)
2. [Installing maple-sim](./docs/2_INSTALLING_MAPLE_SIM.md)
3. [Using the Simulated Arena](./docs/3_USING_THE_SIMULATED_ARENA.md)
4. [Simulating Swerve Drivetrain](./docs/4_SIMULATING_SWERVE_DRIVETRAIN.md)
5. [Simulating Intake](./docs/5_SIMULATING_INTAKE.md)
6. [Simulating Opponent Robots](./docs/6_SIMULATING_OPPONENT_ROBOTS.md)

# Examples and Templates

1. [Advanced Swerve Drive Project](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main/templates/AdvantageKit_AdvancedSwerveDriveProject): 6328's swerve drive project utilizing AdvantageKit, modified to have advanced drivetrain simulations.  The changelog from the original project is [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/1667aa80170e6733d8eaee866da0297e762402fa). 
2. [Maple-Swerve-Skeleton](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton): Our custom swerve drive project based on Advanced Swerve Drive Project with drivetrain simulation, vision simulation and some convenient controlling features.
3. [5516-2024-OffSeason-RobotCode](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/tree/main/example/5516-2024-OffSeason): Our 2024 off-season robot code that implements a variety of advanced simulations.  It can run on real robot and be played like a video game.  [Videos](https://www.youtube.com/watch?v=5jr1L8xWpog&list=PLFS6A0KifAK1ycwlzIlvvFJkWNsQHVjSN)

> ❌ YAGSL is not yet supported since simulation is running inside the library instead of the user code.
