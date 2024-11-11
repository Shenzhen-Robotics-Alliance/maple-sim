package org.ironmaple.simulation.motorsims;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public sealed interface ControlRequest {
    Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity);

    record CurrentOut(Current current) implements ControlRequest {

        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
            return configs.calculateVoltage(current, encoderVelocity);
        }
    }

    record PositionCurrent(Angle setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
            Current current =
                    Amps.of(configs.positionCurrentController.calculate(encoderPosition.in(Radians), setPoint.in(Radians)));
            Voltage currentVoltage = configs.calculateVoltage(current, encoderVelocity);
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(encoderVelocity.in(RadiansPerSecond)));
            return currentVoltage.plus(feedforwardVoltage);
        }
    }

    record PositionVoltage(Angle setPoint) implements ControlRequest {

        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
            Voltage voltage = Volts.of(
                    configs.positionVoltageController.calculate(encoderPosition.in(Radians), setPoint.in(Radians)));
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(
                    configs.calculateVelocity(configs.calculateCurrent(encoderVelocity, voltage), voltage)
                            .in(RadiansPerSecond)));
            return feedforwardVoltage.plus(voltage);
        }
    }

    record VelocityCurrent(AngularVelocity setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
            Current current = Amps.of(configs.velocityCurrentController.calculate(
                    encoderVelocity.in(RadiansPerSecond), setPoint.in(RadiansPerSecond)));
            Voltage currentVoltage = configs.calculateVoltage(current, encoderVelocity);
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(setPoint.in(RadiansPerSecond)));
            return currentVoltage.plus(feedforwardVoltage);
        }
    }

    record VelocityVoltage(AngularVelocity setPoint) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
            Voltage feedbackVoltage = Volts.of(configs.velocityVoltageController.calculate(
                    encoderVelocity.in(RadiansPerSecond), setPoint.in(RadiansPerSecond)));
            Voltage feedforwardVoltage = Volts.of(configs.feedforward.calculate(setPoint.in(RadiansPerSecond)));
            return feedbackVoltage.plus(feedforwardVoltage.copy());
        }
    }

    record VoltageOut(Voltage voltage) implements ControlRequest {
        @Override
        public Voltage updateSignal(SimMotorConfigs configs, Angle encoderPosition, AngularVelocity encoderVelocity) {
            return voltage;
        }
    }
}
