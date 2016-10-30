package pathplanner.common;


public class Obstacle2D {
    public final Pos2D topLeftCorner;
    public final Pos2D bottomRightCorner;
    
    public Obstacle2D(Pos2D topLeftCorner, Pos2D bottomRightCorner){
        
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
    }

}
