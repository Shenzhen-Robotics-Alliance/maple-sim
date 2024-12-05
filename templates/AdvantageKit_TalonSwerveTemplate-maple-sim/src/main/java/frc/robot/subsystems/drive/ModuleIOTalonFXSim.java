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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.CTREMotorSimUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation simulation;

    public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
        super(constants);

        this.simulation = simulation;
        simulation.useDriveMotorController(
                new CTREMotorSimUtil.TalonFXMotorControllerSim(driveTalon, constants.DriveMotorInverted));
        simulation.useSteerMotorController(new CTREMotorSimUtil.TalonFXMotorControllerWithRemoteCancoderSim(
                driveTalon, constants.DriveMotorInverted, cancoder, constants.SteerMotorInverted));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryTimestamps = CTREMotorSimUtil.getSimulationOdometryTimeStamps();

        inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveEncoderUnGearedPositions())
                .mapToDouble(angle -> angle.in(edu.wpi.first.units.Units.Radians))
                .toArray();

        inputs.odometryTurnPositions = Arrays.stream(simulation.getCachedSteerRelativeEncoderPositions())
                .map(Rotation2d::new)
                .toArray(Rotation2d[]::new);
    }
}
