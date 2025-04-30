package org.ironmaple.simulation;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public abstract class goal implements SimulatedArena.Simulatable {
    
    protected Rectangle xyBox;
    protected final Distance height;
    protected final Distance elevation;

    protected final Class gamePieceType;

    protected final Translation3d position;
    protected final SimulatedArena arena;
    protected final int max;
    public final boolean isBlue;
    protected int gamePieceCount=0;

    public goal(SimulatedArena arena, Distance xDimension, Distance yDimension, Distance height, Class gamePieceType, Translation3d position, boolean isBlue, int max){

        xyBox = new Rectangle(xDimension.in(Units.Meters), yDimension.in(Units.Meters));
        this.height = height;
        this.gamePieceType=gamePieceType;
        this.position=position;
        this.arena=arena;
        this.max=max;
        this.elevation=position.getMeasureZ();
        this.isBlue=isBlue;

        xyBox.translate(new Vector2(position.getX(), position.getY()));
    }

    public goal(SimulatedArena arena, Distance xDimension, Distance yDimension, Distance height, Class gamePieceType, Translation3d position, boolean isBlue){
        this(arena, xDimension, yDimension, height, gamePieceType, position, isBlue, 99999);
    }
        
        
    @Override
    public void simulationSubTick(int subTickNum) {
        Set<GamePieceProjectile> gamePiecesLaunched = arena.gamePieceLaunched();
        Set<GamePieceProjectile> toRemoves = new HashSet<>();
        
        for (GamePieceProjectile gamePieceLaunched : gamePiecesLaunched){
            checkPiece(gamePieceLaunched, toRemoves);
        }

        for (GamePieceProjectile toRemove : toRemoves) gamePiecesLaunched.remove(toRemove);
    }

    protected void checkPiece(GamePieceProjectile gamePiece, Set<GamePieceProjectile> toRemove) {
        if (gamePieceType.isAssignableFrom(gamePiece.getClass())&&gamePieceCount!=max){
            if (!toRemove.contains(gamePiece) && checkCollision(gamePiece)){
                gamePieceCount++;
                addPoints();
                toRemove.add(gamePiece);
            }
        }
    }

    protected boolean checkCollision(GamePieceProjectile gamePiece){
        return 
            xyBox.contains(new Vector2(gamePiece.getPose3d().getX(), gamePiece.getPose3d().getX())) &&
            gamePiece.getPose3d().getZ()>=elevation.in(Units.Meters) &&
            gamePiece.getPose3d().getZ()<=elevation.in(Units.Meters) + height.in(Units.Meters);
    }


    public void clear(){
        this.gamePieceCount=0;
    }
    

    protected abstract void addPoints();
    public abstract void draw(List<Pose3d> drawList);


}
