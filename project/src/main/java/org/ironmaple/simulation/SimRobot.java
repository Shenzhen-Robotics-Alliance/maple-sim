package org.ironmaple.simulation;

import java.util.ArrayList;
import java.util.Optional;

import org.ironmaple.simulation.GamePiece.GamePieceVariant;
import org.ironmaple.simulation.IntakeSimulation.IntakeBehavior;
import org.ironmaple.simulation.SimulationArena.SimulationTiming;
import org.ironmaple.simulation.drivesims.DriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SimRobot {
    private final static class Once<T> {
        private final String name;
        private Optional<T> value = Optional.empty();

        public Once(String name) {
            this.name = name;
        }

        public T getUnchecked() {
            return value.get();
        }

        public T get() {
            if (value.isEmpty()) {
                throw new IllegalStateException(name + "is not initialized");
            }
            return getUnchecked();
        }

        public void initialize(T value) {
            if (this.value.isPresent()) {
                throw new IllegalStateException(name + "is already initialized");
            }
            RuntimeLog.debug("Initialized " + name);
            this.value = Optional.of(value);
        }

        public boolean isInitialized() {
            return value.isPresent();
        }
    }

    private final SimulationArena arena;
    private final Once<DriveTrainSimulation> driveTrain = new Once<>("DriveTrainSimulation");
    private final Once<GamePieceStorage> gamePieceStorage = new Once<>("GamePieceStorage");
    private final ArrayList<IntakeSimulation> intakes = new ArrayList<>();
    private final ArrayList<MechanismSim> mechanisms = new ArrayList<>();

    protected SimRobot(SimulationArena arena) {
        this.arena = arena;
    }

    public SimulationTiming timing() {
        return arena.timing;
    }

    void simulationSubTick() {
        if (driveTrain.isInitialized()) {
            driveTrain.get().simulationSubTick();
        }
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

    public <T extends DriveTrainSimulation, C extends DriveTrainSimulationConfig<T, C>> SimRobot initializeDrivetrain(C config) {
        driveTrain.initialize(DriveTrainSimulation.createDriveTrainSimulation(this, config, new Pose2d()));
        arena.world().addBody(driveTrain.getUnchecked());
        return this;
    }

    public <T extends DriveTrainSimulation, C extends DriveTrainSimulationConfig<T, C>> SimRobot initializeDrivetrain(C config, Pose2d initialPose) {
        driveTrain.initialize(DriveTrainSimulation.createDriveTrainSimulation(this, config, initialPose));
        arena.world().addBody(driveTrain.getUnchecked());
        return this;
    }

    public SimRobot initializeGamePieceStorage(int numPieces) {
        gamePieceStorage.initialize(new GamePieceStorage(() -> driveTrain.get().getSimulatedDriveTrainPose3d(), numPieces));
        return this;
    }

    public SimRobot initializeGamePieceStorage(Transform3d firstTransform, Transform3d... otherTransforms) {
        gamePieceStorage.initialize(new GamePieceStorage(() -> driveTrain.get().getSimulatedDriveTrainPose3d(), firstTransform, otherTransforms));
        return this;
    }

    public IntakeSimulation createIntake(Rectangle2d boundingBox, IntakeBehavior behavior, GamePieceVariant... acceptedGamePieceVariants) {
        var intake = new IntakeSimulation(
            driveTrain.get(),
            gamePieceStorage.get(),
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
        return (T) driveTrain.get();
    }

    public GamePieceStorage getGamePieceStorage() {
        return gamePieceStorage.get();
    }
}
