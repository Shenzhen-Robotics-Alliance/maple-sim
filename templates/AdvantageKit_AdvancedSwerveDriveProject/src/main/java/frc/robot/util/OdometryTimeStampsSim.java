package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import org.ironmaple.simulation.SimulatedArena;

public class OdometryTimeStampsSim {
  public static double[] getTimeStamps() {
    final double[] odometryTimestamps = new double[5];
    for (int i = 0; i < 5; i++)
      odometryTimestamps[i] =
          Timer.getFPGATimestamp() - Robot.defaultPeriodSecs + SimulatedArena.SIMULATION_DT * i;
    return odometryTimestamps;
  }
}
