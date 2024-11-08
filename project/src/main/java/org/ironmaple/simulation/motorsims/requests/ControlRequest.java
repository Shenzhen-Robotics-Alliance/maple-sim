package org.ironmaple.simulation.motorsims.requests;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

public sealed interface ControlRequest
        permits CurrentOut, PositionCurrent, PositionVoltage, VelocityCurrent, VelocityVoltage, VoltageOut {
    Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity);
}
