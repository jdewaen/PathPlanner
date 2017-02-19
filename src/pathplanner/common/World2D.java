package pathplanner.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class World2D {

    private ArrayList<Region2D> regions = new ArrayList<Region2D>();
    
    private final Pos2D maxPos;
    private final Pos2D minPos;
    
    public World2D(Pos2D maxPos){
        this(new Pos2D(0, 0), maxPos);
    }
    
    
    public World2D(Pos2D minPos, Pos2D maxPos){
        this.maxPos = maxPos;
        this.minPos = minPos;
    }
    
    
    public void addRegion(Region2D obs){
//        if(isInside(obs.bottomRightCorner) && isInside(obs.topLeftCorner)){
            regions.add(obs);
//        }else{
//            throw new IllegalArgumentException("Obstacle is not inside the world boundaries");
//        }
        
    }
   
    
    public List<Region2D> getRegions(){
        return (List<Region2D>) Collections.unmodifiableList(regions);
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
    
    public boolean isInside(Region2D region){
        return isInside(region.bottomRightCorner) && isInside(region.topLeftCorner);
    }
    
    public boolean intersectsAnyObstacle(Pos2D pos1, Pos2D pos2){
        for(Region2D reg : regions){
            if(reg.intersects(pos1, pos2, 0)){
                return true;
            }
        }
        return false;
    }

}
