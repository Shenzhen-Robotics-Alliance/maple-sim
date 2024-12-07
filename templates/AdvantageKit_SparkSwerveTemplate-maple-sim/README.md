# AdvantageKit SparkMax Swerve Template with maple-sim

[Original Project](https://github.com/Mechanical-Advantage/AdvantageKit/releases/download/v4.0.0-beta-1/AdvantageKit_SparkSwerveTemplate.zip)

This is the AdvantageKit Swerve Template with REV SparkMax hardware, enhanced with maple-sim integration for improved chassis physics simulation.

Not many changes were made to the original project—only the necessary ones to implement maple-sim. See the [changelog from the original project](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/17cf2894008e4fa45492efa3937123494879ef81).

The control loops of the chassis run on REV motor controllers.  During simulation, they are replaced with WpiLib `PIDController`, so there might be slight differences between the real and simulated robot.

## Getting Started
The usage of this project is identical to the original. See the [AdvantageKit Online Documentation](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template) for instructions on how to use this template.

## Running the Simulation
Change `Constants.currentRobotMode` to `SIM` and run `simulateJava`.
Connect the robot from AdvantageScope, open [AdvantageScope Simulation Layout](./AdvantageScope%20Simulation.json), and drive your robot around!

## Using Vision
The [AdvantageKit Vision Example](https://docs.advantagekit.org/getting-started/template-projects/vision-template) is also included in this project. Note that there is a slight difference when setting up vision if you're using maple-sim. See the [code](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/c7f0c858c68351f91118f2042e88d0f0ef40d865/templates/AdvantageKit_SparkSwerveTemplate-maple-sim/src/main/java/frc/robot/RobotContainer.java#L86-L91).
