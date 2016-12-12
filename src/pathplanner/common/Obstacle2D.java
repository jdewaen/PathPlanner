package pathplanner.common;

import java.io.Serializable;


public class Obstacle2D extends Region2D implements Serializable{
    
    public Obstacle2D(Pos2D topLeftCorner, Pos2D bottomRightCorner) {
        super(topLeftCorner, bottomRightCorner);
    }
    public Obstacle2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double startTime, double endTime, double height) {
        super(topLeftCorner, bottomRightCorner, startTime, endTime, height);
    }
    public Obstacle2D(Pos2D topLeftCorner, Pos2D bottomRightCorner,  double height) {
        super(topLeftCorner, bottomRightCorner, height);
    }
    public boolean isObstacle(){
        return true;
    }

}
