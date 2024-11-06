package org.ironmaple.simulation.motorsims.requests;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public record VelocityCurrent(AngularVelocity angularVelocity) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, SimMotorState state) {
        Current current = Amps.of(configs.velocityCurrentController.calculate(
                state.angularVelocity().in(RadiansPerSecond),
                angularVelocity.in(RadiansPerSecond)
        ));
        Voltage currentVoltage = configs.calculateVoltage(current, state.angularVelocity());
        Voltage feedforwardVoltage = configs.feedforward.calculate(angularVelocity);
        return currentVoltage.plus(feedforwardVoltage);
    }
}
