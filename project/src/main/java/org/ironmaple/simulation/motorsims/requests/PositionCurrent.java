package org.ironmaple.simulation.motorsims.requests;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

public record PositionCurrent(Angle setPoint) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, SimMotorState state) {
        Current current = Amps.of(
                configs.positionCurrentController.calculate(state.angularPosition().in(Radians), setPoint.in(Radians))
        );
        Voltage currentVoltage = configs.calculateVoltage(current, state.angularVelocity());
        Voltage feedforwardVoltage = configs.feedforward.calculate(state.angularVelocity());
        return currentVoltage.plus(feedforwardVoltage);
    }
}
