# Advantage Kit - Advanced Swerve Drive Project

### With more enhanced swerve drive simulation using [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim).
Original Project is [here](https://www.chiefdelphi.com/t/advantagekit-2024-log-replay-again/442968/54).

To use the simulation in your own project, please refer to this [Commit Change-Log](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/commit/1667aa80170e6733d8eaee866da0297e762402fa).
To see how to simulate intake for your robot, please refer to this [Intake Example](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/AdvantageKit_AdvancedSwerveDriveProject/src/main/java/frc/robot/subsystems/IntakeExample.java).

When running the simulation, you can see the simulation results through AdvantageScope, please drag the following fields:

- Open field `AdvantageKit/RealOutputs/FieldSimulation/RobotPosition` make it `2024 KitBot`
- Open field `AdvantageKit/RealOutputs/FieldSimulation/Notes` and make it `Note`
- Open field `AdvantageKit/RealOutputs/Intake/NoteInIntake` and make it `Note`
- Open field `AdvantageKit/RealOutputs/Odometry/Robot` and make it `Green Ghost`

Now you can drive the robot around and try grabbing notes with left trigger, and clear the note in intake with left bumper.
