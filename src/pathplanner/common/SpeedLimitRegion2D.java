package pathplanner.common;


public class SpeedLimitRegion2D extends Region2D {
    
    public final double speed;

    public SpeedLimitRegion2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double speed) {
        super(topLeftCorner, bottomRightCorner);
        this.speed = speed;
    }
    
    public boolean isSpeedLimit(){
        return true;
    }
}
