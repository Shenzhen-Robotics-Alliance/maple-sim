<p align="center" markdown>
  ![team logo](./media/team_logo.png){ width="20%"}
  ![project logo](./media/icon.png){ width="79%"}
</p>

### **Elevating FRC Java Robot Simulations to the Next Level with Physics Engines**

??? question "Why a Physics Engine?"
    <p>
        A simulation engine is a powerful tool that provides realistic approximations of physical systems. With 
        <strong>maple-sim</strong>, we integrate the open-source Java rigid-body dynamics engine, 
        <a href="https://github.com/dyn4j/dyn4j">dyn4j</a>, capable of simulating 2D forces and collisions 
        between rigid shapes. This integration transforms the scope of robot simulations by enabling realistic 
        interactions between robots, field elements, and game pieces.
    </p>
    <img src="./media/physics%20engine.png" alt="physics engine illustration" />
    <p>
        Before <strong>maple-sim</strong>, most FRC robot simulations focused solely on the robot itself—its 
        sensors, movements, and internal operations. Now, through the power of physics simulation, 
        <strong>maple-sim</strong> allows your robot to engage directly with its environment. Imagine testing 
        robot interactions with obstacles, field elements, and game pieces, all within the simulated world. 
        A simulation that is realistic enough to <strong>feel like a video game.</strong>
    </p>
    <a href="https://www.youtube.com/watch?v=CBx1_Dosgec">
        <img src="./media/demo%20video%20cover.png" alt="Demo Video 1">
    </a>
    <p>With this advanced level of simulation, the possibilities are endless. You can:</p>
    <ul>
        <li>Test autonomous modes with pinpoint accuracy.</li>
        <li>Fine-tune advanced TeleOp enhancement features like pathfinding-auto-alignment.</li>
        <li>Optimize shooters and other subsystems, all while gathering meaningful data from simulated physics.</li>
    </ul>
    <h3><strong>And the best part? You can achieve all of this without needing a real robot on hand!</strong></h3>

## Simulation Details

For an in-depth description of the simulations, please refer to [Simulation Details](./simulation-details.md)

## Getting Started With Templates

<div class="grid cards" markdown>
-   **CTRE Generated Swerve Code**
    
    ---
    The [CTRE Generated Swerve Code](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html), enhanced with maple-sim integration for improved chassis physics simulation. 

    [:octicons-arrow-right-24: Project Page](https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim/)

-   **YAGSL**

    ---
    Maple-sim is officially included in the amazing [Yet Another Generic Swervedrive Library (YAGSL)](https://github.com/BroncBotz3481/YAGSL) for 2025!

    [:octicons-arrow-right-24: YAGSL 2025 release](https://github.com/Shenzhen-Robotics-Alliance/YAGSL-maple-sim)

-   **AdvantageKit SparkMax/Flex Swerve Template**

    ---
    The [AdvantageKit Swerve Template with REV SparkMax/SparkFlex hardware](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template), enhanced with maple-sim integration for improved chassis physics simulation.
    
    [:octicons-arrow-right-24: Project Page](https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-SparkSwerveTemplate-MapleSim/)

-   **AdvantageKit Talon Swerve Template**

      --- 
      The [AdvantageKit Swerve Template with CTRE hardware](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template), enhanced with maple-sim integration for improved chassis physics simulation.
    
      [:octicons-arrow-right-24: Project Page](https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim/)
    
-   **AdvantageKit Talon Swerve Template - *Enhanced Version***

    ---    
    A further enhanced version of the *TalonSwerveTemplate-maple-sim* project, utilizing [Phoenix 6 simulation](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html) to simulate CTRE motor controller closed-loops and the CAN bus
    
    [:octicons-arrow-right-24: Project Page](https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/)
</div>

??? example "Other Custom Templates & Examples"
    **[Maple-Swerve-Skeleton](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton)**: Our custom swerve drive project based on the Advanced Swerve Drive Project, featuring drivetrain simulation, vision simulation, and convenient control features.
    

    **[5516-2024-OffSeason-RobotCode](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/tree/main/example/5516-2024-OffSeason)**: Our 2024 off-season robot code, which implements a range of advanced simulations. This code can be run on a real robot and even played like a video game. Watch the [Videos](https://www.youtube.com/watch?v=5jr1L8xWpog&list=PLFS6A0KifAK1ycwlzIlvvFJkWNsQHVjSN)

## Online Documentation

Please follow the guides below for quick setup:

1. [Installing maple-sim](./installing-maple-sim.md)
2. [Using the Simulated Arena](./using-the-simulated-arena.md)
3. [Simulating Swerve Drivetrain](./swerve-simulation-overview.md)
4. [Simulating Intake](./simulating-intake.md)
5. [Simulating Projectiles](./simulating-projectiles.md)
6. [Simulating Opponent Robots](./simulating-opponent-robots.md)

!!! info ""
    🙏  Big thanks to [@GrahamSH-LLK](https://github.com/GrahamSH-LLK) for all the help in setting up the online documentation.

## Java Docs
[:octicons-arrow-right-24: Go to javadocs](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/).
!!! info ""
    🙏  Big thanks to [@nstrike](https://www.chiefdelphi.com/u/nstrike/summary) for all the help in setting up the Java Docs.

## Reporting Bugs, Developing and Contributing

!!! bug "Reporting Bugs"
    If you've encountered a bug while using maple-sim in your robot code, please [submit an issue](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/issues/new/choose) and select the "Bug Report" option.  We review issues regularly and will respond as quickly as possible.

!!! example "Suggestions & Improvements"
    - If you have an idea for a new feature, please [submit an issue](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/issues/new/choose) and select the "Feature Request" option.
    - If you think the API for an existing feature could be improved for better readability or usability, please [submit an issue](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/issues/new/choose) and select the "API Enhancement" option.

!!! success "Contributing"
    For detailed guidelines on contributing to the project, please refer to the [contribution guide](./CONTRIBUTION.md).