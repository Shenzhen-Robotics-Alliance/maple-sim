<p align="center" markdown>
  ![team logo](./media/team_logo.png){ width="20%"}
  ![project logo](./media/icon.png){ width="79%"}
</p>

### Elevating FRC Java Robot Simulations to the Next Level with Physics Engines

## Why a Physics Engine?
A simulation engine is a powerful tool that provides realistic approximations of physical systems. With **maple-sim**, we integrate the open-source Java rigid-body dynamics engine, [dyn4j](https://github.com/dyn4j/dyn4j), capable of simulating 2D forces and collisions between rigid shapes. This integration transforms the scope of robot simulations by enabling realistic interactions between robots, field elements, and game pieces.

![physics engine illustration](./media/physics%20engine.png)

Before **maple-sim**, most FRC robot simulations focused solely on the robot itself—its sensors, movements, and internal operations. 
Now, through the power of physics simulation, **maple-sim** allows your robot to engage directly with its environment. 
Imagine testing robot interactions with obstacles, field elements, and game pieces, all within the simulated world.
A simulation that is realistic enough to **feel like a video game.**

[![Demo Video 1](./media/demo%20video%20cover.png)](https://www.youtube.com/watch?v=CBx1_Dosgec)


With this advanced level of simulation, the possibilities are endless. You can:

- Test autonomous modes with pinpoint accuracy.
- Fine-tune advanced TeleOp enhancement features like pathfinding-auto-alignment.
- Optimize shooters and other subsystems, all while gathering meaningful data from simulated physics.

**And the best part? You can achieve all of this without needing a real robot on hand.**

## Simulation Details

For an in-depth description of the simulations, please refer to [Simulation Details](https://shenzhen-robotics-alliance.github.io/maple-sim/0_SIMULATION_DETIALS.html).

## Quick Start

Please follow the guides below for quick setup:

1. [Installing maple-sim](./installing-maple-sim.md)
2. [Using the Simulated Arena](./using-the-simulated-arena.md)
3. [Simulating Swerve Drivetrain](./swerve-simulation-overview.md)
4. [Simulating Intake](./simulating-intake.md)
5. [Simulating Projectiles](./simulating-projectiles.md)
6. [Simulating Opponent Robots](./simulating-opponent-robots.md)

# Java Docs
> 🙏  Big thanks to [nstrike](https://www.chiefdelphi.com/u/nstrike/summary) for all the help in setting up the Java Docs.
[Official Java Docs](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/).

## Examples and Templates

- [Advanced Swerve Drive Project with maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main/templates/AdvantageKit_AdvancedSwerveDriveProject): 6328's swerve drive project utilizing AdvantageKit, modified with advanced drivetrain simulations. View the changelog from the original project [here](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/1667aa80170e6733d8eaee866da0297e762402fa).
- [Maple-Swerve-Skeleton](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton): Our custom swerve drive project based on the Advanced Swerve Drive Project, featuring drivetrain simulation, vision simulation, and convenient control features.
- [Base-Talon-Swerve with maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main/templates/BaseTalonSwerve-maple-sim): Base-Talon-Swerve, modified with advanced drivetrain simulation. This is an example implementation of the [Simplified Swerve Simulation](https://shenzhen-robotics-alliance.github.io/maple-sim/3.1_SWERVE_SIM_EZ_MODE.html).
- [YAGSL-2025-beta](https://github.com/BroncBotz3481/YAGSL-Example/tree/beta): Official 2025-beta release of the amazing [Yet Another Generic Swerve Drive Library](https://www.chiefdelphi.com/t/yet-another-generic-swerve-library-yagsl-v1-release/450844), with maple-sim implemented for enhanced drivetrain simulation.
- [5516-2024-OffSeason-RobotCode](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/tree/main/example/5516-2024-OffSeason): Our 2024 off-season robot code, which implements a range of advanced simulations. This code can be run on a real robot and even played like a video game. Watch the [Videos](https://www.youtube.com/watch?v=5jr1L8xWpog&list=PLFS6A0KifAK1ycwlzIlvvFJkWNsQHVjSN)


## Bugs Developing and Contributing

- If you've encountered a bug while using maple-sim in your robot code, please [submit an issue](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/issues/new/choose) and select the "Bug Report" option.  We review issues regularly and will respond as quickly as possible.

- If you have an idea for a new feature, please [submit an issue](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/issues/new/choose) and select the "Feature Request" option.

- If you think the API for an existing feature could be improved for better readability or usability, please [submit an issue](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/issues/new/choose) and select the "API Enhancement" option.

- For detailed guidelines on contributing to the project, please refer to the [contribution guide](https://shenzhen-robotics-alliance.github.io/maple-sim/CONTRIBUTION.html).