package org.ironmaple.simulation.motorsims.requests;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public record VelocityVoltage(AngularVelocity setPoint) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, SimMotorState state) {
        Voltage feedbackVoltage = Volts.of(
                configs.velocityVoltageController.calculate(state.angularVelocity().in(RadiansPerSecond))
        );
        Voltage feedforwardVoltage = configs.feedforward.calculate(setPoint);
        return feedbackVoltage.plus(feedforwardVoltage);
    }
}
