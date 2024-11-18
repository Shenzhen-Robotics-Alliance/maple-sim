package org.ironmaple.simulation.motorsims.requests;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

public record VelocityVoltage(AngularVelocity setPoint) implements ControlRequest {
    @Override
    public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
        Voltage feedbackVoltage = Volts.of(configs.velocityVoltageController.calculate(
                encoderVelocity.in(RadiansPerSecond), setPoint.in(RadiansPerSecond)));
        Voltage feedforwardVoltage = configs.feedforward.calculate(setPoint);
        return feedbackVoltage.plus(feedforwardVoltage.copy());
    }
}
