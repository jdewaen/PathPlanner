package pathplanner.common;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;


public abstract class Region2D implements Serializable{
    public final Pos2D bottomRightCorner;
    public final Pos2D topLeftCorner;
    public final double startTime;
    public final double endTime;
    public final double height;
    
    
    public Region2D(Pos2D bottomRightCorner, Pos2D topLeftCorner, double startTime, double endTime, double height){
        
        if(bottomRightCorner == null){
            throw new IllegalArgumentException("Top left corner is null");
        }
        
        if(topLeftCorner == null){
            throw new IllegalArgumentException("Bottom right corner is null");
        }
        
        if(bottomRightCorner.x > topLeftCorner.x || bottomRightCorner.y > topLeftCorner.y){
            throw new IllegalArgumentException("Top left corner needs to be above and left of bottom right corner");
        }
        
        this.bottomRightCorner = bottomRightCorner;
        this.topLeftCorner = topLeftCorner;
        this.startTime = startTime;
        this.endTime = endTime;
        this.height = height;
    }
    
    public Region2D(Pos2D topLeftCorner, Pos2D bottomRightCorner){
        this(topLeftCorner, bottomRightCorner, 0, 9999999, 0);
    }
    
    public Region2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double height){
        this(topLeftCorner, bottomRightCorner, 0, 9999999, height);
    }    
    public boolean isObstacle(){
        return false;
    }
    
    public boolean isSpeedLimit(){
        return false;
    }
    
    public boolean isCheckPoint(){
        return false;
    }
    
    public boolean intersects(Pos2D p1, Pos2D p2){
        if(p1.x > topLeftCorner.x && p2.x > topLeftCorner.x) return false;
        if(p1.x < bottomRightCorner.x && p2.x < bottomRightCorner.x) return false;
        if(p1.y > topLeftCorner.y && p2.y > topLeftCorner.y) return false;
        if(p1.y < bottomRightCorner.y && p2.y < bottomRightCorner.y) return false;    
        
        return true;
    }
    
    public boolean contains(Pos2D pos){
        return (pos.x <= topLeftCorner.x && pos.x >= bottomRightCorner.x && pos.y <= topLeftCorner.y && pos.y >= bottomRightCorner.y); 
    }
    
    public boolean fuzzyContains(Pos2D pos, double size){
        return (pos.x - size <= topLeftCorner.x && pos.x + size >= bottomRightCorner.x && pos.y - size <= topLeftCorner.y && pos.y + size >= bottomRightCorner.y); 
    }    
    public List<Pos2D> getVertices(){
        List<Pos2D> result = new ArrayList<Pos2D>();
        result.add(bottomRightCorner);
        result.add(new Pos2D(topLeftCorner.x, bottomRightCorner.y));
        result.add(topLeftCorner);
        result.add(new Pos2D(bottomRightCorner.x, topLeftCorner.y));
        return result;
    }
}
