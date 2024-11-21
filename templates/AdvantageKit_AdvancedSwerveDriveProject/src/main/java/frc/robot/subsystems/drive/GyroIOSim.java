package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.util.OdometryTimeStampsSim;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
        inputs.odometryYawTimestamps = OdometryTimeStampsSim.getTimeStamps();
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec =
                gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
    }
}
