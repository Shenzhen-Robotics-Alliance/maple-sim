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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOSim extends ModuleIOTalonFX {
    // TODO: use ModuleHardware to store the hardware
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60(1);
    public static final DCMotor TURN_GEARBOX = DCMotor.getFalcon500(1);

    private final DCMotorSim driveMotorSim;
    private final DCMotorSim turnMotorSim;
    private final TalonFXSimState driveTalonSim;
    private final TalonFXSimState turnTalonSim;
    private final CANcoderSimState encoderSim;

    public ModuleIOSim(SwerveModuleConstants constants) {
        super(constants, false);

        this.driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), constants.DriveInertia, constants.DriveMotorGearRatio),
                DRIVE_GEARBOX
        );
        this.turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_GEARBOX
        );

        this.driveTalonSim = super.driveTalon.getSimState();
        this.turnTalonSim = super.turnTalon.getSimState();
        this.encoderSim = super.cancoder.getSimState();
    }
}
