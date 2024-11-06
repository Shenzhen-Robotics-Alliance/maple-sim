package org.ironmaple.simulation.motorsims.requests;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;

public record PositionVoltage(Angle setPoint) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, SimMotorState state) {
        Voltage voltage = Volts.of(configs.positionVoltageController.calculate(
                state.angularPosition().in(Radians), setPoint.in(Radians)));
        Voltage feedforwardVoltage = configs.feedforward.calculate(
                state.angularVelocity(),
                configs.calculateVelocity(configs.calculateCurrent(state.angularVelocity(), voltage), voltage));
        return feedforwardVoltage.plus(voltage);
    }
}
