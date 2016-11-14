package pathplanner.common;


public abstract class Region2D {
    public final Pos2D topLeftCorner;
    public final Pos2D bottomRightCorner;
    public final double startTime;
    public final double endTime;
    
    
    public Region2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double startTime, double endTime){
        
        if(topLeftCorner == null){
            throw new IllegalArgumentException("Top left corner is null");
        }
        
        if(bottomRightCorner == null){
            throw new IllegalArgumentException("Bottom right corner is null");
        }
        
        if(topLeftCorner.x > bottomRightCorner.x || topLeftCorner.y > bottomRightCorner.y){
            throw new IllegalArgumentException("Top left corner needs to be above and left of bottom right corner");
        }
        
        this.topLeftCorner = topLeftCorner;
        this.bottomRightCorner = bottomRightCorner;
        this.startTime = startTime;
        this.endTime = endTime;
    }
    
    public Region2D(Pos2D topLeftCorner, Pos2D bottomRightCorner){
        this(topLeftCorner, bottomRightCorner, 0, 9999999);
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
}
