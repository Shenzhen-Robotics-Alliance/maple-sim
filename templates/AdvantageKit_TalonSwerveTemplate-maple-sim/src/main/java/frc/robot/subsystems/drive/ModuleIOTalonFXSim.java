// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation swerveModuleSimulation;
    private final TalonFXSimState driveTalonSim;
    private final TalonFXSimState turnTalonSim;
    private final CANcoderSimState encoderSim;

    public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation swerveModuleSimulation) {
        super(constants);

        this.swerveModuleSimulation = swerveModuleSimulation;
        swerveModuleSimulation.requestDriveControl();
        swerveModuleSimulation.requestSteerControl();

        this.driveTalonSim = super.driveTalon.getSimState();
        this.turnTalonSim = super.turnTalon.getSimState();
        this.encoderSim = super.cancoder.getSimState();
    }

    public void updateSimulation() {
        this.driveTalonSim
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.odometryTimestamps = new double[] {};
    }
}
