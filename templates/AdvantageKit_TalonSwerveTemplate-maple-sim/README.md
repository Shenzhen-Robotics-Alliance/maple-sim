# AdvantageKit Talon Swerve Template with maple-sim

[Original Project](https://github.com/Mechanical-Advantage/AdvantageKit/releases/download/v4.0.0-beta-1/AdvantageKit_TalonFXSwerveTemplate.zip)

This is the AdvantageKit Swerve Template with CTRE hardware, enhanced with maple-sim integration for improved chassis physics simulation.

Not many changes were made to the original project—only the necessary ones to implement maple-sim. See the [changelog from the original project](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/9c010d3ee9037b16c2a7a462ea68a09af3814441).

The control loops of the chassis run on CTRE motor controllers.  During simulation, they are replaced with WpiLib `PIDController`, so there might be slight differences between the real and simulated robot.

For a more precise simulation of CTRE motor controllers, see [AdvantageKit Talon Swerve Template with Advanced Phoenix Simulation](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/tree/main/templates/AdvantageKit_TalonSwerveTemplate_EnhancedPhoenixSimulation).

## Getting Started
The usage of this project is identical to the original. See the [AdvantageKit Online Documentation](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template) for instructions on how to use this template.

## Running the Simulation
Change `Constants.currentRobotMode` to `SIM` and run `simulateJava`.
Connect the robot from AdvantageScope, open [AdvantageScope Simulation Layout](./AdvantageScope%20Simulation.json), and drive your robot around!

## Using Vision
The [AdvantageKit Vision Example](https://docs.advantagekit.org/getting-started/template-projects/vision-template) is also included in this project. Note that there is a slight difference when setting up vision if you're using maple-sim. See the [code](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/87588b4fb0554383b3881a4b0f56147519c32c68/templates/AdvantageKit_TalonSwerveTemplate-maple-sim/src/main/java/frc/robot/RobotContainer.java#L100-L105).
