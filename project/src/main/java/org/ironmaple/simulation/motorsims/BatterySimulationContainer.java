package org.ironmaple.simulation.motorsims;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class BatterySimulationContainer {
    private static BatterySimulationContainer instance = null;

    public static BatterySimulationContainer getInstance() {
        if (instance == null) instance = new BatterySimulationContainer();
        return instance;
    }

    private final List<Supplier<Current>> electricalAppliances = new ArrayList<>();
    private double batteryVoltage = 12.0;

    public void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        this.electricalAppliances.add(customElectricalAppliances);
    }

    public void addMotor(MapleMotorSim mapleMotorSim) {
        this.electricalAppliances.add(mapleMotorSim::getStatorCurrent);
    }

    public void flush() {
        batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(electricalAppliances.stream()
                .mapToDouble(currentSupplier -> currentSupplier.get().in(Units.Amps))
                .toArray());

        RoboRioSim.setVInVoltage(batteryVoltage);
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double constrainVoltage(double voltage) {
        return MathUtil.clamp(voltage, -batteryVoltage, batteryVoltage);
    }
}
