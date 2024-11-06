package org.ironmaple.simulation.motorsims.requests;

import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;

public sealed interface ControlRequest permits CurrentOut, PositionCurrent, PositionVoltage, VelocityCurrent, VelocityVoltage, VoltageOut {
    Voltage updateSignal(SimMotorConfigs configs, SimMotorState state);
}
