package org.ironmaple.simulation.motorsims.requests;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

public record PositionVoltage(Angle setPoint) implements ControlRequest {

    @Override
    public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
        Voltage voltage = Volts.of(
                configs.positionVoltageController.calculate(encoderPosition.in(Radians), setPoint.in(Radians)));
        Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(
                configs.calculateVelocity(configs.calculateCurrent(encoderVelocity, voltage), voltage)
                        .in(RadiansPerSecond)));
        return feedforwardVoltage.plus(voltage);
    }
}
