package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 *
 *
 * <h1>Simulates the main battery of the robot.</h1>
 *
 * <p>This class simulates the behavior of a robot's battery. Electrical appliances can be added to the battery to draw
 * current. The battery voltage is affected by the current drawn from various appliances.
 */
public class SimulatedBattery {
    // Nominal voltage for a fully charged battery (13.5 volts).
    private static final double BATTERY_NOMINAL_VOLTAGE = 13.5;

    // Filter to smooth the current readings.
    private static final LinearFilter currentFilter = LinearFilter.movingAverage(50);

    private static final List<Supplier<Current>> electricalAppliances = new ArrayList<>();

    // The current battery voltage in volts.
    private static double batteryVoltageVolts = BATTERY_NOMINAL_VOLTAGE;

    private static boolean disableBatterySim = false;

    /** Disables the battery simulation. This is a lazy quick fix to help the opponent simulation. */
    public static void disableBatterySim() {
        disableBatterySim = true;
        electricalAppliances.clear();
        batteryVoltageVolts = BATTERY_NOMINAL_VOLTAGE;
    }

    /**
     *
     *
     * <h2>Adds a custom electrical appliance.</h2>
     *
     * <p>Connects the electrical appliance to the battery, allowing it to draw current from the battery.
     *
     * @param customElectricalAppliances The supplier for the current drawn by the appliance.
     */
    public static void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        electricalAppliances.add(customElectricalAppliances);
    }

    /**
     *
     *
     * <h2>Adds a motor to the list of electrical appliances.</h2>
     *
     * <p>The motor will draw current from the battery.
     *
     * @param mapleMotorSim The motor simulation object.
     */
    public static void addMotor(MapleMotorSim mapleMotorSim) {
        electricalAppliances.add(mapleMotorSim::getSupplyCurrent);
    }

    /**
     *
     *
     * <h2>Updates the battery simulation.</h2>
     *
     * <p>Calculates the battery voltage based on the current drawn by all appliances.
     *
     * <p>The battery voltage is clamped to avoid going below the brownout voltage.
     */
    public static void simulationSubTick() {
        double totalCurrentAmps = getTotalCurrentDrawn().in(Amps);
        totalCurrentAmps = currentFilter.calculate(totalCurrentAmps);

        if (Double.isNaN(batteryVoltageVolts)) {
            batteryVoltageVolts = 12.0;
            DriverStation.reportError(
                    "[MapleSim] Internal Library Error: Calculated battery voltage is invalid"
                            + ", reverting to normal operation voltage...",
                    false);
        }
        if (batteryVoltageVolts < RoboRioSim.getBrownoutVoltage()) {
            batteryVoltageVolts = RoboRioSim.getBrownoutVoltage();
            DriverStation.reportError("[MapleSim] BrownOut Detected, protecting battery voltage...", false);
        }

        /// Quick fix to lock battery simulation to nominal voltage.
        if (!disableBatterySim) {
            batteryVoltageVolts =
                    BatterySim.calculateLoadedBatteryVoltage(BATTERY_NOMINAL_VOLTAGE, 0.02, totalCurrentAmps);
        }

        RoboRioSim.setVInVoltage(batteryVoltageVolts);
        SmartDashboard.putNumber("BatterySim/TotalCurrent (Amps)", totalCurrentAmps);
        SmartDashboard.putNumber("BatterySim/BatteryVoltage (Volts)", batteryVoltageVolts);
    }

    /**
     *
     *
     * <h2>Obtains the voltage of the battery.</h2>
     *
     * @return The battery voltage as a {@link Voltage} object.
     */
    public static Voltage getBatteryVoltage() {
        return Volts.of(batteryVoltageVolts);
    }

    /**
     *
     *
     * <h2>Obtains the total current drawn from the battery.</h2>
     *
     * <p>Iterates through all the appliances to obtain the total current used.
     *
     * @return The total current as a {@link Current} object.
     */
    public static Current getTotalCurrentDrawn() {
        double totalCurrentAmps = electricalAppliances.stream()
                .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
                .sum();
        return Amps.of(totalCurrentAmps);
    }

    /**
     *
     *
     * <h2>Clamps the voltage according to the supplied voltage and the battery's capabilities.</h2>
     *
     * <p>If the supplied voltage exceeds the battery's maximum voltage, it will be reduced to match the battery's
     * voltage.
     *
     * @param voltage The voltage to be clamped.
     * @return The clamped voltage as a {@link Voltage} object.
     */
    public static Voltage clamp(Voltage voltage) {
        return Volts.of(MathUtil.clamp(voltage.in(Volts), -batteryVoltageVolts, batteryVoltageVolts));
    }
}
