package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.gamepieces.GamePieceInterface;

public abstract class goal implements SimulatedArena.Simulatable {

    protected Rectangle xyBox;
    protected final Distance height;
    protected final Distance elevation;

    protected final String gamePieceType;

    protected final Translation3d position;
    protected final SimulatedArena arena;
    protected final int max;
    public final boolean isBlue;
    protected int gamePieceCount = 0;
    protected Rotation3d peiceAngle = null;
    // protected Rotation3d peiceVelAngle = null;
    protected Angle peiceAngleTolerence = Angle.ofBaseUnits(15, Units.Degrees);
    // protected Angle peiceVelAngleTolerence = Angle.ofBaseUnits(15, Units.Degrees);

    public goal(
            SimulatedArena arena,
            Distance xDimension,
            Distance yDimension,
            Distance height,
            String gamePieceType,
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
            String gamePieceType,
            Translation3d position,
            boolean isBlue) {
        this(arena, xDimension, yDimension, height, gamePieceType, position, isBlue, 99999);
    }

    @Override
    public void simulationSubTick(int subTickNum) {

        // System.out.println(gamePieceType);

        Set<GamePieceInterface> gamePiecesLaunched =
                new HashSet<GamePieceInterface>(arena.getPeicesByType(gamePieceType));
        Set<GamePieceInterface> toRemoves = new HashSet<>();

        for (GamePieceInterface gamePieceLaunched : gamePiecesLaunched) {
            checkPiece(gamePieceLaunched, toRemoves);
        }

        for (GamePieceInterface toRemove : toRemoves) this.arena.removePeice(toRemove);
    }

    public void setNeededAngle(Rotation3d angle, Angle angleTolerence) {
        peiceAngle = angle;
        peiceAngleTolerence = angleTolerence;
    }

    public void setNeededAngle(Rotation3d angle) {
        setNeededAngle(angle, peiceAngleTolerence);
    }

    // public void setNeededVelAngle(Rotation3d velAngle, Angle peiceVelAngleTolerence) {
    //     peiceVelAngle = velAngle;
    //     this.peiceVelAngleTolerence = peiceVelAngleTolerence;
    // }

    // public void setNeededVelAngle(Rotation3d velAngl) {
    //     setNeededVelAngle(velAngl, peiceVelAngleTolerence);
    // }

    protected void checkPiece(GamePieceInterface gamePiece, Set<GamePieceInterface> toRemove) {
        if (gamePieceCount != max) {

            if (!toRemove.contains(gamePiece) && this.checkCollision(gamePiece)) {
                gamePieceCount++;
                this.addPoints();
                System.out.println("Game peice hit");
                System.out.println(this);
                toRemove.add(gamePiece);
            }
        }
    }

    protected boolean checkCollision(GamePieceInterface gamePiece) {
        // System.out.println(checkRotationAndVel(gamePiece));
        // System.out.println(xyBox.contains(
        //         new Vector2(gamePiece.getPose3d().getX(), gamePiece.getPose3d().getY())));
        // System.out.println(gamePiece.getPose3d().getZ() >= elevation.in(Units.Meters));
        // System.out.println(gamePiece.getPose3d().getZ() <= elevation.in(Units.Meters) + height.in(Units.Meters));
        return xyBox.contains(new Vector2(
                        gamePiece.getPose3d().getX(), gamePiece.getPose3d().getY()))
                && gamePiece.getPose3d().getZ() >= elevation.in(Units.Meters)
                && gamePiece.getPose3d().getZ() <= elevation.in(Units.Meters) + height.in(Units.Meters)
                && checkRotationAndVel(gamePiece);
    }

    protected boolean checkRotationAndVel(GamePieceInterface gamePiece) {
        // System.out.println("Cycle");
        // System.out.println(
        //         rotationsEqualWithinTolerence(peiceAngle, gamePiece.getPose3d().getRotation(), peiceAngleTolerence));
        // System.out.println(rotationsEqualWithinTolerence(
        //         peiceVelAngle, transToRotation(gamePiece.getVelocity3dMPS()), peiceVelAngleTolerence));

        // System.out.println(peiceVelAngle.getX());
        // System.out.println(peiceVelAngle.getY());
        // System.out.println(peiceVelAngle.getZ());
        // System.out.println(gamePiece.getVelocity3dMPS());
        // System.out.println(transToRotation(gamePiece.getVelocity3dMPS()).getX());
        // System.out.println(transToRotation(gamePiece.getVelocity3dMPS()).getY());
        // System.out.println(transToRotation(gamePiece.getVelocity3dMPS()).getZ());
        // System.out.println(this.getClass());
        // System.out.println(peiceVelAngle
        //         .minus(transToRotation(gamePiece.getVelocity3dMPS()))
        //         .getX());
        // System.out.println(peiceVelAngle
        //         .minus(transToRotation(gamePiece.getVelocity3dMPS()))
        //         .getY());
        // System.out.println(peiceVelAngle
        //         .minus(transToRotation(gamePiece.getVelocity3dMPS()))
        //         .getZ());
        // System.out.println(peiceVelAngle
        //         .minus(transToRotation(gamePiece.getVelocity3dMPS()))
        //         .getAngle());
        // System.out.println(peiceVelAngleTolerence.in(Units.Degrees));

        return checkRotation(gamePiece) && checkVel(gamePiece);
    }

    public static Rotation3d transToRotation(Translation3d toConvert) {
        return new Rotation3d(
                Math.atan2(floor2Decimals(toConvert.getZ()), floor2Decimals(toConvert.getY())),
                Math.atan2(floor2Decimals((toConvert.getX())), floor2Decimals((toConvert.getY()))),
                Math.atan2(floor2Decimals(toConvert.getZ()), floor2Decimals(toConvert.getX())));
    }

    protected static double floor2Decimals(double toRound) {
        return Math.floor(toRound * 100) / 100;
    }

    // protected static boolean rotationsEqualWithinTolerence(
    //         Rotation3d rotation1, Rotation3d rotation2, Angle tolerence) {
    //     if (rotation1 == null || rotation2 == null) return true;

    //     return rotation1.minus(rotation2).getMeasureAngle().in(Units.Degrees) < tolerence.in(Units.Degrees);
    //     // return (Math.abs(rotation.getX()) <= Math.toRadians(toleranceDegrees))
    //     //         // || Math.abs(rotation.getX()) - Math.PI <= Math.toRadians(toleranceDegrees))
    //     //         && (Math.abs(rotation.getY()) <= Math.toRadians(toleranceDegrees))
    //     //         // || Math.abs(rotation.getY()) - Math.PI <= Math.toRadians(toleranceDegrees))
    //     //         && (Math.abs(rotation.getZ()) <= Math.toRadians(toleranceDegrees));
    //     // // || Math.abs(rotation.getZ()) - Math.PI <= Math.toRadians(toleranceDegrees));
    // }

    protected boolean checkRotation(GamePieceInterface gamePiece) {
        if (peiceAngle == null) return true;

        // System.out.println("new");

        // System.out.println(gamePiece.getPose3d().getRotation().getX());
        // System.out.println(gamePiece.getPose3d().getRotation().getY());
        // System.out.println(gamePiece.getPose3d().getRotation().getZ());

        // System.out.println("break");

        // System.out.println(peiceAngle.getX());
        // System.out.println(peiceAngle.getY());
        // System.out.println(peiceAngle.getZ());

        // System.out.println("break");
        // Rotation3d idealRotation = new Rotation3d(0, peiceAngle.getY(), peiceAngle.getZ());
        // Rotation3d difference = idealRotation.minus(gamePiece.getPose3d().getRotation());

        // System.out.println(difference.getX());
        // System.out.println(difference.getY());
        // System.out.println(difference.getZ());

        // System.out.println("break");

        // System.out.println(difference.getMeasureAngle().in(Units.Degrees));
        // System.out.println(peiceAngleTolerence.in(Units.Degrees));
        // System.out.println(difference.getMeasureAngle().in(Units.Degrees) < peiceAngleTolerence.in(Units.Degrees)
        //         || (difference.getMeasureAngle().in(Units.Degrees) < peiceAngleTolerence.in(Units.Degrees) + 180
        //                 && difference.getMeasureAngle().in(Units.Degrees)
        //                         > 180 - peiceAngleTolerence.in(Units.Degrees)));

        // System.out.println("end");

        // System.out.println(gamePiece
        //         .getPose3d()
        //         .getRotation()
        //         .minus(peiceAngle)
        //         .getMeasureAngle()
        //         .in(Units.Degrees));

        // System.out.println(gamePiece.getPose3d().getRotation().getY());
        // System.out.println(gamePiece.getPose3d().getRotation().getZ());
        // System.out.println(peiceAngle.getY());
        // System.out.println(peiceAngle.getZ());
        // System.out.println(peiceAngleTolerence.in(Units.Degrees));
        // Angle difference = gamePiece.getPose3d().getRotation().minus(peiceAngle).getMeasureAngle();
        // System.out.println(difference.in(Units.Degrees));

        System.out.println(gamePiece.getPose3d().getRotation().getY());
        System.out.println(gamePiece.getPose3d().getRotation().getZ());
        System.out.println(peiceAngle.getY());
        System.out.println(peiceAngle.getZ());
        System.out.println(gamePiece.getPose3d().getRotation().unaryMinus().getY());
        System.out.println(gamePiece.getPose3d().getRotation().unaryMinus().getZ());

        System.out.println();

        System.out.println(gamePiece.getPose3d().getRotation().minus(peiceAngle).getY());
        System.out.println(gamePiece.getPose3d().getRotation().minus(peiceAngle).getZ());

        System.out.println(gamePiece
                .getPose3d()
                .getRotation()
                .unaryMinus()
                .minus(peiceAngle)
                .getY());

        System.out.println(gamePiece
                .getPose3d()
                .getRotation()
                .unaryMinus()
                .minus(peiceAngle)
                .getZ());

        System.out.println();

        System.out.println(gamePiece
                .getPose3d()
                .getRotation()
                .minus(peiceAngle)
                .getMeasureAngle()
                .in(Units.Degrees));
        System.out.println(gamePiece
                .getPose3d()
                .getRotation()
                .unaryMinus()
                .minus(peiceAngle)
                .getMeasureAngle()
                .in(Units.Degrees));

        System.out.println(gamePiece
                .getPose3d()
                .getRotation()
                .rotateBy(gamePiece.getPose3d().getRotation().unaryMinus())
                .getX());
        System.out.println(gamePiece
                .getPose3d()
                .getRotation()
                .rotateBy(gamePiece.getPose3d().getRotation().unaryMinus())
                .getY());
        System.out.println("\n\n\n");

        return gamePiece
                                .getPose3d()
                                .getRotation()
                                .minus(peiceAngle)
                                .getMeasureAngle()
                                .in(Units.Degrees)
                        < peiceAngleTolerence.in(Units.Degrees)
                || gamePiece
                                .getPose3d()
                                .getRotation()
                                .unaryMinus()
                                .minus(peiceAngle)
                                .getMeasureAngle()
                                .in(Units.Degrees)
                        < peiceAngleTolerence.in(Units.Degrees);
    }

    protected boolean checkVel(GamePieceInterface gamePiece) {
        return true;
        // if (peiceVelAngle == null) {
        //     System.out.println(this.getClass());
        //     return true;
        // }

        // Rotation3d peiceVelRotation = new Rotation3d(
        //         0,
        //         Math.atan2(
        //                 gamePiece.getVelocity3dMPS().getX(),
        //                 gamePiece.getVelocity3dMPS().getY()),
        //         Math.atan2(
        //                 gamePiece.getVelocity3dMPS().getZ(),
        //                 Math.hypot(
        //                         gamePiece.getVelocity3dMPS().getX(),
        //                         gamePiece.getVelocity3dMPS().getY())));

        // System.out.println("new");

        // System.out.println(peiceVelRotation.getX());
        // System.out.println(peiceVelRotation.getY());
        // System.out.println(peiceVelRotation.getZ());

        // System.out.println("break");

        // System.out.println(peiceVelAngle.getX());
        // System.out.println(peiceVelAngle.getY());
        // System.out.println(peiceVelAngle.getZ());

        // System.out.println("break");
        // Rotation3d idealRotation = new Rotation3d(0, peiceVelAngle.getY(), peiceVelAngle.getZ());
        // Rotation3d difference = idealRotation.minus(peiceVelRotation);

        // System.out.println(difference.getX());
        // System.out.println(difference.getY());
        // System.out.println(difference.getZ());

        // System.out.println("break");

        // System.out.println(difference.getMeasureAngle().in(Units.Degrees));
        // System.out.println(peiceVelAngleTolerence.in(Units.Degrees));
        // System.out.println(difference.getMeasureAngle().in(Units.Degrees) < peiceVelAngleTolerence.in(Units.Degrees)
        //         || (difference.getMeasureAngle().in(Units.Degrees) < peiceVelAngleTolerence.in(Units.Degrees) + 180
        //                 && difference.getMeasureAngle().in(Units.Degrees)
        //                         > 180 - peiceVelAngleTolerence.in(Units.Degrees)));

        // System.out.println("end");
        // return difference.getMeasureAngle().in(Units.Degrees) < peiceVelAngleTolerence.in(Units.Degrees)
        //         || (difference.getMeasureAngle().in(Units.Degrees) < peiceVelAngleTolerence.in(Units.Degrees) + 180
        //                 && difference.getMeasureAngle().in(Units.Degrees)
        //                         > 180 - peiceVelAngleTolerence.in(Units.Degrees));
    }

    public void clear() {
        this.gamePieceCount = 0;
    }

    public boolean isFull() {
        return this.gamePieceCount == this.max;
    }

    protected abstract void addPoints();

    public abstract void draw(List<Pose3d> drawList);
}
