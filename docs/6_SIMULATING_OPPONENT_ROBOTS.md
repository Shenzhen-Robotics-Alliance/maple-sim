## Simulating Opponent Robots

#### This document is based on the [AIRobotInSimulation class](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/AIRobotInSimulation.java) from Maple-Swerve-Skeleton.  Check the source for a more detailed understanding.

> ⚠️ **Note**
>
> You are reading the documentation for a Beta version of maple-sim. API references are subject to change in future versions.

### 0. Overview
Opponent robots can be added to the field for realistic driver practicing.  They can be programmed to do the following:

- Automatically cycle acrross the field.  This can help the driver practice offense skills when there are other robots on field.
- Automatically running feed-cycles to deliver feed-shot notes.  This can help the driver practice front-field clean up and feeder strategy.
- Controlled by another joystick to play defense.  This can help the drivers practice defense and counter-defense skills.

### 1. Creating and managing opponent robots

Opponent robots are also simulated using the `SimplifiedSwerveDriveSimulation` class.  We need to create a container class that manages the instances.



<div style="display:flex">
    <h3 style="width:49%"><< Prev: <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/5_SIMULATING_PROJECTILES.html">Simulating Projectiles</a></h3>
</div>