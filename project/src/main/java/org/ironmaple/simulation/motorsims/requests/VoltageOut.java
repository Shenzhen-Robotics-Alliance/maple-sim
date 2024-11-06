package org.ironmaple.simulation.motorsims.requests;

import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;

public record VoltageOut(Voltage voltage) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, SimMotorState state) {
        return voltage;
    }
}
