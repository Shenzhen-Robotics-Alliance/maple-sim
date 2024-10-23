// A drop-in replacement for WPILib's DCMotorSim to simulate modern brushless motors
// Copyright (C) 2024 Team-5516-"Iron Maple"
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Original source: https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/src/main/java/org/ironmaple/simulation/drivesims/BrushlessMotorSim.java

package org.ironmaple.simulation.drivesims;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * <h1>DCMotorSim with Additional Features for Simulating Modern Brushless Motors.</h1>
 *
 * <p>By Team 5516 "IRON MAPLE".</p>
 *
 * <p>This class extends the functionality of the original `DCMotorSim` to provide additional simulations specific to modern brushless motors. It simulates the following aspects:</p>
 * <ul>
 *   <li>Friction force on the rotor.</li>
 *   <li>Smart current limiting.</li>
 *   <li>Brake and coast modes for brushless motors.</li>
 *   <li>Simulated encoder readings.</li>
 * </ul>
 * */
public class BrushlessMotorSim {
    private final DCMotor motor;
    private final double gearRatio, frictionTorque, loadInertiaJKgMetersSquared;

    private double requestedVoltageOutput, currentLimitAmps;
    private boolean breakModeEnabled;

    private double angularPositionRad, angularVelocityRadPerSec, appliedVolts, currentDrawAmps;

    /**
     * <h2>Constructs a Brushless Motor Simulation Instance.</h2>
     *
     * @param motor the {@link DCMotor} model representing the motor(s) in the simulation
     * @param gearRatio the gear ratio of the mechanism; values greater than 1 indicate a reduction
     * @param loadIntertiaJKgMetersSquared the rotational inertia of the mechanism, in kg·m²
     * @param frictionVoltage the voltage required to overcome friction and make the mechanism move
     * */
    public BrushlessMotorSim(DCMotor motor, double gearRatio, double loadIntertiaJKgMetersSquared, double frictionVoltage) {
        this.motor = motor;
        this.gearRatio = gearRatio;
        this.frictionTorque = motor.getTorque(motor.getCurrent(0, frictionVoltage)) * gearRatio;

        this.loadInertiaJKgMetersSquared = loadIntertiaJKgMetersSquared;
        disableCurrentLimit();
        this.breakModeEnabled = true;

        this.angularPositionRad = angularVelocityRadPerSec = appliedVolts = currentDrawAmps = 0;
    }


    /**
     * <h2>Requests an Output Voltage for the Motor.</h2>
     *
     * <p><strong>Note:</strong> The requested voltage may not always be fully applied due to current limits. For details on current limiting, refer to {@link #enableCurrentLimit(double)}.</p>
     *
     * @param volts the requested output voltage for the motor
     * */
    public void requestVoltageOutput(double volts) {
        this.requestedVoltageOutput = volts;
    }

    /**
     * <h2>Configures a Current Limit for the Motor.</h2>
     *
     * <p>If the motor's supply current exceeds the specified limit, the motor will reduce its output to prevent exceeding the limit.</p>
     *
     * @param currentLimitAmps the maximum allowed current, in amperes
     * */
    public void enableCurrentLimit(double currentLimitAmps) {
        this.currentLimitAmps = currentLimitAmps;
    }

    /**
     * <h2>Disables the Current Limit of the Motor.</h2>
     *
     * <p>This method removes the current limit, allowing the motor to operate without any current restrictions.</p>
     * */
    public void disableCurrentLimit() {
        this.currentLimitAmps = Double.POSITIVE_INFINITY;
    }

    /**
     * <h2>Configures the Zero Power Behavior of the Motor.</h2>
     *
     * <p>This method sets the motor's zero power behavior, similar to the <code>setZeroPowerBehavior()</code> method for brushless motors.</p>
     * <p>When enabled, the motor is set to brake mode; when disabled, the motor is set to coast mode.</p>
     *
     * @param enabled <code>true</code> to enable brake mode, <code>false</code> to enable coast mode
     * */
    public void setMotorBrakeEnabled(boolean enabled) {
        this.breakModeEnabled = enabled;
    }

    /**
     * <h2>Sets the Current State of the Motor.</h2>
     *
     * <p>This method instantly shifts the motor to a given state, setting its angular position and velocity.</p>
     * <p>Equivalent to the {@link DCMotorSim#setState(double, double)} method.</p>
     *
     * @param angularPositionRad the angular position of the motor, in radians
     * @param angularVelocityRadPerSec the angular velocity of the motor, in radians per second
     * */
    public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
        this.angularPositionRad = angularPositionRad;
        this.angularVelocityRadPerSec = angularVelocityRadPerSec;
    }

    /**
     * <h2>Updates the Simulation.</h2>
     *
     * <p>This method steps the motor simulation forward by a given time interval (<code>dt</code>), recalculating and storing the states of the motor.</p>
     * <p>Equivalent to {@link DCMotorSim#update(double)}.</p>
     *
     * @param dtSeconds the time step for the simulation, in seconds
     * */
    public void update(double dtSeconds) {
        appliedVolts = constrainOutputVoltage(
                motor,
                angularVelocityRadPerSec * gearRatio,
                currentLimitAmps,
                requestedVoltageOutput
        );

        double totalTorque = getMotorElectricTorque();

        /* apply friction force */
        final boolean electricTorqueResultingInAcceleration = totalTorque * angularVelocityRadPerSec > 0;
        if (electricTorqueResultingInAcceleration)
            totalTorque = MathUtil.applyDeadband(totalTorque, frictionTorque, Double.POSITIVE_INFINITY);
        else
            totalTorque += getCurrentFrictionTorque();

        this.angularVelocityRadPerSec += totalTorque / loadInertiaJKgMetersSquared * dtSeconds;
        this.angularPositionRad += this.angularVelocityRadPerSec * dtSeconds;
    }

    /**
     * <h2>Calculates the Electric Torque on the Rotor.</h2>
     *
     * @return the torque applied to the mechanism by the motor's electrical output
     * */
    private double getMotorElectricTorque() {
        if (!breakModeEnabled && appliedVolts == 0)
            return currentDrawAmps = 0;
        currentDrawAmps = motor.getCurrent(
                angularVelocityRadPerSec * gearRatio,
                appliedVolts
        );
        return motor.getTorque(currentDrawAmps) * gearRatio;
    }

    /**
     * <h2>Calculates the Dynamic Friction Torque on the Mechanism.</h2>
     *
     * <p>This method simulates the amount of dynamic friction acting on the rotor when the mechanism is rotating.</p>
     * <p>In the real world, dynamic friction torque is typically constant. However, in the simulation, it is made proportional to the rotor speed when the speed is small to avoid oscillation.</p>
     *
     * @return the amount of dynamic friction torque on the mechanism, in Newton-meters
     * */
    private double getCurrentFrictionTorque() {
        final double kFriction = 3.0,
                percentAngularVelocity = Math.abs(angularVelocityRadPerSec) * gearRatio / motor.freeSpeedRadPerSec,
                currentFrictionTorqueMagnitude = Math.min(percentAngularVelocity * kFriction * frictionTorque, frictionTorque);
        return Math.copySign(currentFrictionTorqueMagnitude, -angularVelocityRadPerSec);
    }

    /**
     * <h2>Constrains the Output Voltage of the Motor.</h2>
     *
     * <p>This method constrains the output voltage of the motor to ensure it operates within the current limit and does not exceed the available voltage from the robot's system.</p>
     * <p>The output voltage is constrained such that the motor does not function outside of the specified current limit. Additionally, it ensures that the voltage does not exceed the robot's input voltage, which can be obtained from {@link RoboRioSim#getVInVoltage()}.</p>
     *
     * @param motor the {@link DCMotor} that models the motor
     * @param motorCurrentVelocityRadPerSec the current velocity of the motor's rotor (geared), in radians per second
     * @param currentLimitAmps the configured current limit, in amperes. You can configure the current limit using {@link #enableCurrentLimit(double)}.
     * @param requestedOutputVoltage the requested output voltage
     *
     * @return the constrained output voltage based on the current limit and system voltage
     * */
    public static double constrainOutputVoltage(DCMotor motor, double motorCurrentVelocityRadPerSec, double currentLimitAmps, double requestedOutputVoltage) {
        final double currentAtRequestedVolts = motor.getCurrent(
                motorCurrentVelocityRadPerSec,
                requestedOutputVoltage
        );

        /* normally, motor controller starts cutting the supply voltage when the current exceed 120% the current limit */
        final boolean currentTooHigh = Math.abs(currentAtRequestedVolts) > 1.2 * currentLimitAmps;
        double limitedVoltage = requestedOutputVoltage;
        if (currentTooHigh) {
            final double limitedCurrent = Math.copySign(currentLimitAmps, currentAtRequestedVolts);
            limitedVoltage = motor.getVoltage(
                    motor.getTorque(limitedCurrent),
                    motorCurrentVelocityRadPerSec
            );
        }

        if (Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltage))
            limitedVoltage = requestedOutputVoltage;

        return MathUtil.clamp(limitedVoltage, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage());
    }

    /**
     * <h2>Obtains the Voltage Actually Applied to the Motor.</h2>
     *
     * <p>This method returns the voltage that is actually applied to the motor after being constrained by {@link #constrainOutputVoltage(DCMotor, double, double, double)}.</p>
     * <p>The voltage is constrained to ensure that the current limit is not exceeded and that it does not surpass the robot's battery voltage.</p>
     *
     * @return the constrained voltage actually applied to the motor
     * */
    public double getAppliedVolts() {
        return appliedVolts;
    }

    /**
     * <h2>Obtains the Angular Position of the Mechanism.</h2>
     *
     * <p>This method is equivalent to {@link DCMotorSim#getAngularPositionRad()}.</p>
     *
     * @return the final angular position of the mechanism, in radians
     * */
    public double getAngularPositionRad() {
        return angularPositionRad;
    }

    /**
     * <h2>Obtains the Angular Position of the Motor.</h2>
     *
     * @return the un-geared angular position of the motor (encoder reading), in radians
     * */
    public double getEncoderPositionRad() {
        return angularPositionRad * gearRatio;
    }

    /**
     * <h2>Obtains the Angular Velocity of the Mechanism.</h2>
     *
     * <p>This method is equivalent to {@link DCMotorSim#getAngularVelocityRadPerSec()}.</p>
     *
     * @return the final angular velocity of the mechanism, in radians per second
     * */
    public double getAngularVelocityRadPerSec() {
        return angularVelocityRadPerSec;
    }

    /**
     * <h2>Obtains the Angular Velocity of the Encoder.</h2>
     *
     * @return the un-geared angular velocity of the motor (encoder velocity), in radians per second
     * */
    public double getEncoderVelocityRadPerSec() {
        return angularVelocityRadPerSec * gearRatio;
    }

    /**
     * <h2>Obtains the Amount of Current Flowing into the Motor.</h2>
     *
     * <p>This method is equivalent to {@link DCMotorSim#getCurrentDrawAmps()}.</p>
     *
     * @return the amount of current flowing into the motor, in amperes
     * */
    public double getCurrentDrawAmps() {
        return currentDrawAmps;
    }
}
