package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class SimulatedArena {
    public static final int SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;
    public static final double SIMULATION_DT = TimedRobot.kDefaultPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD;

    protected final World<Body> physicsWorld;
    protected final AbstractDriveTrainSimulation mainRobot;
    protected final Set<AbstractDriveTrainSimulation> driveTrainSimulations;
    protected final Set<GamePieceOnFieldSimulation> gamePieces;

    public SimulatedArena(AbstractDriveTrainSimulation mainRobot, FieldObstaclesMap obstaclesMap) {
        this.physicsWorld = new World<>();
        this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
        for (Body obstacle: obstaclesMap.obstacles)
            this.physicsWorld.addBody(obstacle);
        this.mainRobot = mainRobot;
        this.driveTrainSimulations = new HashSet<>();
        addDriveTrainSimulation(mainRobot);
        this.gamePieces = new HashSet<>();
    }

    public void simulationPeriodic() {
        final long t0 = System.nanoTime();
        competitionPeriodic();
        final double subPeriodSeconds = LoggedRobot.defaultPeriodSecs / SIMULATION_SUB_TICKS_IN_1_PERIOD;
        // move through a few sub-periods in each update
        for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++) {
            for (AbstractDriveTrainSimulation driveTrainSimulation:driveTrainSimulations)
                driveTrainSimulation.simulationPeriodic(i, subPeriodSeconds);
            this.physicsWorld.step(1, subPeriodSeconds);
        }

        Logger.recordOutput(
                "MapleArenaSimulation/Dyn4j Simulator CPU Time MS",
                (System.nanoTime() - t0) / 1000000.0
        );
    }

    public void addDriveTrainSimulation(AbstractDriveTrainSimulation driveTrainSimulation) {
        this.physicsWorld.addBody(driveTrainSimulation);
        this.driveTrainSimulations.add(driveTrainSimulation);
    }

    public void addGamePiece(GamePieceOnFieldSimulation gamePiece) {
        this.physicsWorld.addBody(gamePiece);
        this.gamePieces.add(gamePiece);
    }

    public void removeGamePiece(GamePieceOnFieldSimulation gamePiece) {
        this.physicsWorld.removeBody(gamePiece);
        this.gamePieces.remove(gamePiece);
    }

    public void clearGamePieces() {
        for (GamePieceOnFieldSimulation gamePiece: this.gamePieces)
            this.physicsWorld.removeBody(gamePiece);
        this.gamePieces.clear();
    }

    public void resetFieldForAuto() {
        clearGamePieces();
        placeGamePiecesOnField();
    }

    /**
     * do field reset by placing all the game-pieces on field(for autonomous)
     * */
    public abstract void placeGamePiecesOnField();

    /**
     * update the score counts & human players periodically
     * implement this method in current year's simulation
     * */
    public abstract void competitionPeriodic();

    /**
     * stores the obstacles on a competition field, which includes the border and the game pieces
     * */
    public static abstract class FieldObstaclesMap {
        private final List<Body> obstacles = new ArrayList<>();

        protected void addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
            final Body obstacle = getObstacle(Geometry.createSegment(
                    GeometryConvertor.toDyn4jVector2(startingPoint),
                    GeometryConvertor.toDyn4jVector2(endingPoint)
            ));
            obstacles.add(obstacle);
        }

        protected void addRectangularObstacle(double width, double height, Pose2d pose) {
            final Body obstacle = getObstacle(Geometry.createRectangle(
                    width, height
            ));

            obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose));
            obstacles.add(obstacle);
        }

        private static Body getObstacle(Convex shape) {
            final Body obstacle = new Body();
            obstacle.setMass(MassType.INFINITE);
            final BodyFixture fixture = obstacle.addFixture(shape);
            fixture.setFriction(0.6);
            fixture.setRestitution(0.6);
            return obstacle;
        }
    }
}
