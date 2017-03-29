package pathplanner.common;

import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class World2D {

    private ArrayList<Obstacle2DB> obstacles = new ArrayList<Obstacle2DB>();
    
    protected final Pos2D maxPos;
    protected final Pos2D minPos;
    
    private final Rectangle2D shape;
    
    public World2D(Pos2D maxPos){
        this(new Pos2D(0, 0), maxPos);
    }
    
    
    public World2D(Pos2D minPos, Pos2D maxPos){
        this.maxPos = maxPos;
        this.minPos = minPos;
        
        Pos2D diff = maxPos.minus(minPos);
        this.shape = new Rectangle2D.Double(minPos.x, minPos.y, diff.x, diff.y);
    }
    
    
    public void addObstacle(Obstacle2DB obs){
//        if(isInside(obs.bottomRightCorner) && isInside(obs.topLeftCorner)){
            obstacles.add(obs);
//        }else{
//            throw new IllegalArgumentException("Obstacle is not inside the world boundaries");
//        }
        
    }
   
    
    public List<Obstacle2DB> getObstacles(){
        return (List<Obstacle2DB>) Collections.unmodifiableList(obstacles);
    }
    
    public Pos2D getMinPos(){
        return minPos;
    } 
    
    public Pos2D getMaxPos(){
        return maxPos;
    }
    
    public boolean isInside(Pos2D pos){
        if(pos == null) return false;
        return (pos.x >= minPos.x && pos.x <= maxPos.x && pos.y >= minPos.y &&  pos.y <= maxPos.y);
    }
  
    
    public boolean isInside(Obstacle2DB obstacle){
        return obstacle.shape.intersects(shape);
    }
    
    public boolean isInsideAnyObstacle(Pos2D pos) {
        for (Obstacle2DB obs : obstacles) {
            if (obs.contains(pos)) { return true; }
        }
        return false;
    }
    
    public boolean intersectsAnyObstacle(Pos2D pos1, Pos2D pos2){
        for(Obstacle2DB obs : obstacles){
            if(obs.intersects(pos1, pos2, 0)){
                return true;
            }
        }
        return false;
    }

}
