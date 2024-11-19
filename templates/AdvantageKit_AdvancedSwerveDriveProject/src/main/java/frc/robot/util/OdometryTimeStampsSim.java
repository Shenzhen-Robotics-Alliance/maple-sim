package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import org.ironmaple.simulation.SimulatedArena;

public class OdometryTimeStampsSim {
    public static double[] getTimeStamps() {
        final double[] odometryTimestamps = new double[5];
        for (int i = 0; i < SimulatedArena.getSimulationSubTicksIn1Period(); i++)
            odometryTimestamps[i] = Timer.getFPGATimestamp()
                    - Robot.defaultPeriodSecs
                    + SimulatedArena.getSimulationDt().in(Seconds) * i;
        return odometryTimestamps;
    }
}
