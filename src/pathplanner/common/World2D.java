package pathplanner.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class World2D {
    private ArrayList<Region2D> regions = new ArrayList<Region2D>();
    
    private Pos2D maxPos;
    
    public World2D(Pos2D maxPos){
        this.maxPos = maxPos;
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
   
    public Pos2D getMaxPos(){
        return maxPos;
    }
    
    public boolean isInside(Pos2D pos){
        if(pos == null) return false;
        return (pos.x >= 0 && pos.x <= maxPos.x && pos.y >= 0 &&  pos.y <= maxPos.y);
    }

}
