# 1. Hardware Abstractions

## Before You Begin
> ⚠️ If you're using AdvantageKit, your code is already hardware-abstracted. You do not need to restructure your code to use maple-sim.

## General Idea
> The core idea is that the code for your subsystem interacts with an IO interface that can have multiple implementations.
> 
> The code runs EXACTLY the same regardless of which IO interface it interacts with, whether on a real robot or in a physics engine.

<img src="media%2Fhardware%20abstraction.svg" width="100%">

To clarify:

When running on a real robot, the subsystem code interacts with the hardware through a Hardware IO implementation.
When running in a simulation, the subsystem code interacts with the physics engine via the Simulation IO implementation.

Optionally, you can use the [log-replay technology](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/what-is-advantagekit/index.md) with [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit).

## Subsystem Code Structure

To implement hardware abstraction, you need to organize the code for EACH subsystem according to this structure:

```
subsystems/
├── MySubsystem/
│   ├── MySubsystem.java
│   ├── MySubsystemIO.java
│   ├── MySubsystemIOTalonFX.java
│   ├── MySubsystemIOSparkMax.java
│   └── MySubsystemIOSim.java
```

- `MySubsystem.java`:  The main code that controls the subsystem.
- `MySubsystemIO.java`: Defines the IO interface, specifying the abstract inputs and outputs of the subsystem.
- `MySubsystemIOTalonFX.java`: A hardware implementation of the IO interface using Falcon motors.
- `MySubsystemIOSparkMax.java`: A hardware implementation of the IO interface using Neo motors.
- `MySubsystemIOSim.java`: he simulation implementation of the IO interface, which runs the physics simulation.

Reference: [io-interface.md from Advantage Kit Documents](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/recording-inputs/io-interfaces.md)

<div style="display:flex; justify-content: space-between; width: 100%">
    <h3><< <a href="../README.md">Back to Readme</a></h3>
    <h3>Next: <a href="./2_INSTALLING_MAPLE_SIM.md">Installing maple-sim</a> >></h3>
</div>
