package org.ironmaple.simulation.motorsims.requests;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

public record VelocityCurrent(AngularVelocity setPoint) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
        Current current = Amps.of(configs.velocityCurrentController.calculate(
                encoderVelocity.in(RadiansPerSecond), setPoint.in(RadiansPerSecond)));
        Voltage currentVoltage = configs.calculateVoltage(current, encoderVelocity);
        Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(setPoint.in(RadiansPerSecond)));
        return currentVoltage.plus(feedforwardVoltage);
    }
}
