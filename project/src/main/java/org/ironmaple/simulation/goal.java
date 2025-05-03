package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public abstract class goal implements SimulatedArena.Simulatable {

    protected Rectangle xyBox;
    protected final Distance height;
    protected final Distance elevation;

    protected final Class gamePieceType;

    protected final Translation3d position;
    protected final SimulatedArena arena;
    protected final int max;
    public final boolean isBlue;
    protected int gamePieceCount = 0;
    protected Rotation3d peiceAngle = null;
    protected Rotation3d peiceVelAngle = null;
    protected double peiceAngleTolerence = 15;
    protected double peiceVelAngleTolerence = 15;

    public goal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            Class gamePieceType,
            Translation3d position,
            boolean isBlue,
            int max) {

        xyBox = new Rectangle(xDimension.in(Units.Meters), yDimension.in(Units.Meters));
        this.height = height;
        this.gamePieceType = gamePieceType;
        this.position = position;
        this.arena = arena;
        this.max = max;
        this.elevation = position.getMeasureZ();
        this.isBlue = isBlue;

        xyBox.translate(new Vector2(position.getX(), position.getY()));
    }

    public goal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            Class gamePieceType,
            Translation3d position,
            boolean isBlue) {
        this(arena, xDimension, yDimension, height, gamePieceType, position, isBlue, 99999);
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        Set<GamePieceProjectile> gamePiecesLaunched = arena.gamePieceLaunched();
        Set<GamePieceProjectile> toRemoves = new HashSet<>();

        for (GamePieceProjectile gamePieceLaunched : gamePiecesLaunched) {
            checkPiece(gamePieceLaunched, toRemoves);
        }

        for (GamePieceProjectile toRemove : toRemoves) gamePiecesLaunched.remove(toRemove);
    }

    public void setNeededAngle(Rotation3d angle, double angleTolerence) {
        peiceAngle = angle;
        peiceAngleTolerence = angleTolerence;
    }

    public void setNeededAngle(Rotation3d angle) {
        setNeededAngle(angle, peiceAngleTolerence);
    }

    public void setNeededVelAngle(Rotation3d velAngle, double peiceVelAngleTolerence) {
        peiceVelAngle = velAngle;
        this.peiceVelAngleTolerence = peiceVelAngleTolerence;
    }

    public void setNeededVelAngle(Rotation3d velAngl) {
        setNeededVelAngle(velAngl, peiceVelAngleTolerence);
    }

    protected void checkPiece(GamePieceProjectile gamePiece, Set<GamePieceProjectile> toRemove) {
        if (gamePieceType.isAssignableFrom(gamePiece.getClass()) && gamePieceCount != max) {

            if (!toRemove.contains(gamePiece) && this.checkCollision(gamePiece)) {
                gamePieceCount++;
                this.addPoints();
                System.out.println("Game peice hit");
                System.out.println(this);
                toRemove.add(gamePiece);
            }
        }
    }

    protected boolean checkCollision(GamePieceProjectile gamePiece) {
        System.out.println("ooooh ahhh update");
        System.out.println(xyBox.contains(
                new Vector2(gamePiece.getPose3d().getX(), gamePiece.getPose3d().getX())));
        System.out.println(gamePiece.getPose3d().getZ() >= elevation.in(Units.Meters));
        System.out.println(gamePiece.getPose3d().getZ() <= elevation.in(Units.Meters) + height.in(Units.Meters));
        System.out.println(checkRotationAndVel(gamePiece));
        System.out.println("ooooh ahhh update");

        return xyBox.contains(new Vector2(
                        gamePiece.getPose3d().getX(), gamePiece.getPose3d().getY()))
                && gamePiece.getPose3d().getZ() >= elevation.in(Units.Meters)
                && gamePiece.getPose3d().getZ() <= elevation.in(Units.Meters) + height.in(Units.Meters);
        // && checkRotationAndVel(gamePiece);
    }

    protected boolean checkRotationAndVel(GamePieceProjectile gamePiece) {
        return rotationsEqualWithinTolerence(peiceAngle, gamePiece.getPose3d().getRotation(), peiceAngleTolerence)
                && compRotationAndVectorWithTolerence(
                        peiceVelAngle, gamePiece.getVelocity3dMPS(), peiceVelAngleTolerence);
    }

    protected static boolean rotationsEqualWithinTolerence(
            Rotation3d rotation1, Rotation3d rotation2, double toleranceDegrees) {
        if (rotation1 == null || rotation2 == null) return true;
        Translation3d vector1 = new Translation3d(1, rotation1);

        Translation3d vector2 = new Translation3d(1, rotation2);

        double rotationToleranceVectorNorm = new Translation2d(1, Rotation2d.fromDegrees(toleranceDegrees))
                .minus(new Translation2d(1, 0))
                .getNorm();

        return vector1.minus(vector2).getNorm() < rotationToleranceVectorNorm
                || vector1.times(-1).minus(vector2).getNorm() < rotationToleranceVectorNorm;
    }

    protected static boolean compRotationAndVectorWithTolerence(
            Rotation3d rotation, Translation3d vector, double toleranceDegrees) {
        if (rotation == null || vector == null) return true;
        Translation3d rotationVector = new Translation3d(1, rotation);

        double rotationToleranceVectorNorm = new Translation2d(1, Rotation2d.fromDegrees(toleranceDegrees))
                .minus(new Translation2d(1, 0))
                .getNorm();

        return rotationVector.minus(vector).getNorm() < rotationToleranceVectorNorm
                || rotationVector.times(-1).minus(vector).getNorm() < rotationToleranceVectorNorm;
    }

    public void clear() {
        this.gamePieceCount = 0;
    }

    protected abstract void addPoints();

    public abstract void draw(List<Pose3d> drawList);
}
