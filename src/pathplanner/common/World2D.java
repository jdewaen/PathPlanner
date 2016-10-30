package pathplanner.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class World2D {
    private ArrayList<Obstacle2D> obstacles = new ArrayList<Obstacle2D>();
    
    private Pos2D maxPos;
    
    public World2D(Pos2D maxPos){
        this.maxPos = maxPos;
    }
    
    public void addObstacle(Obstacle2D obs){
        if(isInside(obs.topLeftCorner) && isInside(obs.bottomRightCorner)){
            obstacles.add(obs);
        }else{
            throw new IllegalArgumentException("Obstacle is not inside the world boundaries");
        }
        
    }
    
    public List<Obstacle2D> getObstacles(){
        return (List<Obstacle2D>) Collections.unmodifiableList(obstacles);
    }
    
    public Pos2D getMaxPos(){
        return maxPos;
    }
    
    public boolean isInside(Pos2D pos){
        if(pos == null) return false;
        return (pos.x <= maxPos.x && pos.y <= maxPos.y);
    }

}
