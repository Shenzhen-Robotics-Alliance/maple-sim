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

public class SimulatedBattery {
    private static final double BATTERY_NOMINAL_VOLTAGE = 13.5; // A fully charged battery should be 13.5 volts

    private static final LinearFilter currentFilter = LinearFilter.movingAverage(50);
    private static final List<Supplier<Current>> electricalAppliances = new ArrayList<>();
    private static double batteryVoltageVolts = BATTERY_NOMINAL_VOLTAGE;

    public static void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        electricalAppliances.add(customElectricalAppliances);
    }

    public static void addMotor(MapleMotorSim mapleMotorSim) {
        electricalAppliances.add(mapleMotorSim::getSupplyCurrent);
    }

    public static void flush() {
        double totalCurrentAmps = electricalAppliances.stream()
                        .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
                        .sum();

        totalCurrentAmps = currentFilter.calculate(totalCurrentAmps);

        SmartDashboard.putNumber("BatterySim/TotalCurrentAmps", totalCurrentAmps);
        batteryVoltageVolts = BatterySim.calculateLoadedBatteryVoltage(BATTERY_NOMINAL_VOLTAGE, 0.02, totalCurrentAmps);

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

        RoboRioSim.setVInVoltage(batteryVoltageVolts);
    }

    public static Voltage getBatteryVoltage() {
        return Volts.of(batteryVoltageVolts);
    }

    /**
     * Clamps the voltage to be a voltage between - battery voltage and + battery voltage.
     *
     * @param voltage
     * @return
     */
    public static Voltage clamp(Voltage voltage) {
        return Volts.of(MathUtil.clamp(voltage.in(Volts), -batteryVoltageVolts, batteryVoltageVolts));
    }
}
