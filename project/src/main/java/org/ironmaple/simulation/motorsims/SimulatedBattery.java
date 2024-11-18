package org.ironmaple.simulation.motorsims;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class SimulatedBattery {
    private static SimulatedBattery instance = null;

    public static SimulatedBattery getInstance() {
        if (instance == null) instance = new SimulatedBattery();
        return instance;
    }

    private final List<Supplier<Current>> electricalAppliances = new ArrayList<>();
    private double batteryVoltageVolts = 12.0;

    public void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        this.electricalAppliances.add(customElectricalAppliances);
    }

    public void addMotor(MapleMotorSim mapleMotorSim) {
        this.electricalAppliances.add(mapleMotorSim::getSupplyCurrent);
    }

    public void flush() {
        final double totalCurrentAmps = electricalAppliances.stream()
                .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
                .sum();

        SmartDashboard.putNumber("BatterySim/TotalCurrentAmps", totalCurrentAmps);
        batteryVoltageVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);

        batteryVoltageVolts = MathUtil.clamp(batteryVoltageVolts, 0, 12);

        RoboRioSim.setVInVoltage(batteryVoltageVolts);
    }

    public Voltage getBatteryVoltage() {
        return Volts.of(batteryVoltageVolts);
    }
}
