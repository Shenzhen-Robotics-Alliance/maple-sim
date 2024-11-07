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

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.GamePieceStorage;
import org.ironmaple.simulation.MechanismSim;
import org.ironmaple.simulation.SimRobot;
import org.ironmaple.simulation.MechanismSim.OutputType;
import org.ironmaple.simulation.drivesims.DriveTrainSimulation;
import org.ironmaple.utils.ProjectileUtil;
import org.ironmaple.utils.geometry.Velocity3d;
import org.ironmaple.utils.mathutils.GeometryConvertor;

public class FlywheelIOSim implements FlywheelIO {
    private final MechanismSim sim;
    private final GamePieceStorage gamePieceStorage;
    private final DriveTrainSimulation driveTrain;

    public FlywheelIOSim(final SimRobot robot) {
        this.sim = robot.createMechanism(
            DCMotor.getNEO(1),
            1.5,
            KilogramSquareMeters.of(0.004),
            Volts.of(0.2)
        );
        gamePieceStorage = robot.getGamePieceStorage();
        driveTrain = robot.getDriveTrain();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRadPerSec = sim.getVelocity().in(RadiansPerSecond);
        inputs.appliedVolts = sim.getRotorVoltage().in(Volts);
        inputs.currentAmps = new double[] { sim.getStatorCurrentDraw().baseUnitMagnitude() };
    }

    @Override
    public void setVoltage(double volts) {
        sim.setControl(Volts.of(volts));
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        sim.setControl(OutputType.VOLTAGE, RadiansPerSecond.of(velocityRadPerSec));
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    /**
     * when the intake passes the note to the flywheels, this method is called to simulate launching a
     * note from the shooter
     */
    public void receiveNote() {
        if (sim.getVelocity().in(RotationsPerSecond) < 5.0) {
            // don't pull note
            return;
        }
        Velocity3d velo = new Velocity3d(
            driveTrain.getSimulatedDriveTrainPose3d().getRotation(),
            sim.getVelocity().in(RotationsPerSecond) / 8.0 //arbitrary value
        ).plus(new Velocity3d(GeometryConvertor.toWpilibVelocity2d(driveTrain.getLinearVelocity())));
        gamePieceStorage.pullHighest(true).ifPresent(note -> {
            note.launch(
                driveTrain.getSimulatedDriveTrainPose3d(),
                velo,
                ProjectileUtil.gravity(11.0)
            );
        });
    }
}
