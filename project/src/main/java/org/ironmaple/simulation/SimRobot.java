package org.ironmaple.simulation;

import java.util.ArrayList;

import org.ironmaple.simulation.GamePiece.GamePieceVariant;
import org.ironmaple.simulation.IntakeSimulation.IntakeBehavior;
import org.ironmaple.simulation.SimulationArena.SimulationTiming;
import org.ironmaple.simulation.drivesims.DriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SimRobot {
    private final SimulationArena arena;
    private final DriveTrainSimulation driveTrain;
    private final GamePieceStorage gamePieceStorage;
    private final ArrayList<IntakeSimulation> intakes = new ArrayList<>();
    private final ArrayList<MechanismSim> mechanisms = new ArrayList<>();

    protected <T extends DriveTrainSimulation, C extends DriveTrainSimulationConfig<T, C>> SimRobot(SimulationArena arena, C drivetrainConfig, int gamePieceStorageCapacity) {
        this.arena = arena;
        this.driveTrain = DriveTrainSimulation.createDriveTrainSimulation(this, drivetrainConfig, new Pose2d());
        this.gamePieceStorage = new GamePieceStorage(() -> driveTrain.getSimulatedPose3d(), gamePieceStorageCapacity);
    }

    public SimulationTiming timing() {
        return arena.timing;
    }

    void simulationSubTick() {
        driveTrain.simulationSubTick();
        ArrayList<Double> mechanismCurrents = new ArrayList<>();
        for (var mechanism : mechanisms) {
            mechanism.simulationSubTick();
            mechanismCurrents.add(mechanism.getSupplyCurrent().baseUnitMagnitude());
        }
        double vin = BatterySim.calculateLoadedBatteryVoltage(
                12.2,
                0.015,
                mechanismCurrents.stream().mapToDouble(Double::doubleValue).toArray());
        RoboRioSim.setVInVoltage(vin);
    }

    public IntakeSimulation createIntake(Rectangle2d boundingBox, IntakeBehavior behavior, GamePieceVariant... acceptedGamePieceVariants) {
        var intake = new IntakeSimulation(
            driveTrain,
            gamePieceStorage,
            GeometryConvertor.toDyn4jRectangle(boundingBox),
            behavior,
            acceptedGamePieceVariants
        );
        intakes.add(intake);
        arena.world().addContactListener(intake.getGamePieceContactListener());
        RuntimeLog.debug("Created IntakeSimulation");
        return intake;
    }

    public MechanismSim createMechanism(DCMotor motor, double gearRatio, MomentOfInertia loadIntertia, Voltage frictionVoltage) {
        MechanismSim mapleMotor = new MechanismSim(timing(), motor, gearRatio, loadIntertia, frictionVoltage);
        mechanisms.add(mapleMotor);
        RuntimeLog.debug("Created MechanismSim");
        return mapleMotor;
    }

    @SuppressWarnings("unchecked")
    public <T extends DriveTrainSimulation> T getDriveTrain() {
        return (T) driveTrain;
    }

    public GamePieceStorage getGamePieceStorage() {
        return gamePieceStorage;
    }
}
