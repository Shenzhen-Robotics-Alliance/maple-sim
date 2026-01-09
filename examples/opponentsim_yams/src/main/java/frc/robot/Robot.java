// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot
{

  private Command autonomousCommand;

  public final RobotContainer robotContainer;


  public Robot()
  {
    robotContainer = new RobotContainer();
  }


  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {}


  @Override
  public void disabledExit() {}


  @Override
  public void autonomousInit()
  {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null)
    {
      autonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {}


  @Override
  public void autonomousExit() {}


  @Override
  public void teleopInit()
  {
    if (autonomousCommand != null)
    {
      autonomousCommand.cancel();
    }
  }


  @Override
  public void teleopPeriodic() {}

  @Override
  public void simulationPeriodic() {
      robotContainer.simulationPeriodic();
  }

    @Override
  public void teleopExit() {}


  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }


  @Override
  public void testPeriodic() {}


  @Override
  public void testExit() {}
}
