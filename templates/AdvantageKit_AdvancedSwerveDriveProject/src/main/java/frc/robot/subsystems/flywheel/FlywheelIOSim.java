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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

public class FlywheelIOSim implements FlywheelIO {
    private FlywheelSim sim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.004, 1.5), DCMotor.getNEO(1));
    private PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.positionRad = 0.0;
        // store the rpm of the motor
        this.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(
                inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec());
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        closedLoop = true;
        pid.setSetpoint(velocityRadPerSec);
        this.ffVolts = ffVolts;
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }

    private double velocityRPM = 0.0;

    /**
     * when the intake passes the note to the flywheels, this method is called to simulate launching a note from the
     * shooter
     */
    public void shootNoteWithCurrentRPM(Pose2d robotSimulationWorldPose, ChassisSpeeds chassisSpeedsFieldRelative) {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new NoteOnFly(
                                robotSimulationWorldPose.getTranslation(), // specify the position of the chassis
                                new Translation2d(
                                        0.2, 0), // the shooter is installed at this position on the robot (in reference
                                // to the robot chassis center)
                                chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
                                // to add it to the initial velocity of the projectile
                                robotSimulationWorldPose.getRotation(), // the shooter facing is the robot's facing
                                0.45, // initial height of the flying note
                                velocityRPM
                                        / 6000
                                        * 20, // we think the launching speed is proportional to the rpm, and is 16
                                // meters/second when the motor rpm is 6000
                                Math.toRadians(55) // the note is launched at fixed angle of 55 degrees.
                                )
                        .asSpeakerShotNote(() -> System.out.println("hit target!!!"))
                        .enableBecomeNoteOnFieldAfterTouchGround()
                        .withProjectileTrajectoryDisplayCallBack(
                                (pose3ds) -> Logger.recordOutput(
                                        "Flywheel/NoteProjectileSuccessful", pose3ds.toArray(Pose3d[]::new)),
                                (pose3ds) -> Logger.recordOutput(
                                        "Flywheel/NoteProjectileUnsuccessful", pose3ds.toArray(Pose3d[]::new))));
    }
}
