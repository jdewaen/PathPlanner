package pathplanner.common;


public class Obstacle2D extends Region2D {
    
    public Obstacle2D(Pos2D topLeftCorner, Pos2D bottomRightCorner) {
        super(topLeftCorner, bottomRightCorner);
    }
    public Obstacle2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double startTime, double endTime) {
        super(topLeftCorner, bottomRightCorner, startTime, endTime);
    }
    public boolean isObstacle(){
        return true;
    }

}
