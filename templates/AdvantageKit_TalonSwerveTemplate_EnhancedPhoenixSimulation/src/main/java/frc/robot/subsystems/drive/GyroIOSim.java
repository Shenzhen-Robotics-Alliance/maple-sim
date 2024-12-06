package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.robot.util.CTREMotorSimUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                gyroSimulation.getMeasuredAngularVelocity().in(edu.wpi.first.units.Units.RadiansPerSecond));

        inputs.odometryYawTimestamps = CTREMotorSimUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}
