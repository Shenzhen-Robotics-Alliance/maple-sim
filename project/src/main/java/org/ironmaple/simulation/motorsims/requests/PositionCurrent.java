package org.ironmaple.simulation.motorsims.requests;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

public record PositionCurrent(Angle setPoint) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
        Current current =
                Amps.of(configs.positionCurrentController.calculate(encoderPosition.in(Radians), setPoint.in(Radians)));
        Voltage currentVoltage = configs.calculateVoltage(current, encoderVelocity);
        Voltage feedforwardVoltage = configs.feedforward.calculate(encoderVelocity);
        return currentVoltage.plus(feedforwardVoltage);
    }
}
