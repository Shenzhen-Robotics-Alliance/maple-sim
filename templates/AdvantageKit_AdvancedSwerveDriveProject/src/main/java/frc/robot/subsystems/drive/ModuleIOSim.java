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

// Modified by 5516 "IRON MAPLE", original source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.OdometryTimeStampsSim;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.requests.VoltageOut;

/** Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO */
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveCurrentAmps = new double[] {Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps))};

        inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.turnPosition = Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPosition().in(Radians));
        inputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
        inputs.turnCurrentAmps = new double[] {Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps))};

        inputs.odometryTimestamps = OdometryTimeStampsSim.getTimeStamps();
        inputs.odometryDrivePositionsRad = moduleSimulation.getCachedDriveWheelFinalPositionsRad();
        inputs.odometryTurnPositions = Arrays.stream(moduleSimulation.getCachedSteerRelativeEncoderPositions())
                .mapToObj(Rotation2d::fromRadians)
                .toArray(Rotation2d[]::new);
    }

    @Override
    public void setDriveVoltage(double volts) {
        moduleSimulation.requestDriveOutput(new VoltageOut(Volts.of(volts)));
    }

    @Override
    public void setTurnVoltage(double volts) {
        moduleSimulation.requestSteerControl(new VoltageOut(Volts.of(volts)));
    }
}
