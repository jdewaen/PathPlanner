package pathplanner.common;


public class CheckPoint2D extends Region2D {

    public final double maxTime;
    
    public CheckPoint2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double maxTime) {
        super(topLeftCorner, bottomRightCorner, 0, maxTime);
        this.maxTime = maxTime;
    }
    
    public boolean isCheckPoint(){
        return true;
    }
}
