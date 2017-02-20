package pathplanner.common;


public class SpeedLimitRegion2D extends Region2D {
    
    /**
     * 
     */
    private static final long serialVersionUID = -489244540768222063L;
    public final double speed;

    public SpeedLimitRegion2D(Pos2D topLeftCorner, Pos2D bottomRightCorner, double speed) {
        super(topLeftCorner, bottomRightCorner);
        this.speed = speed;
    }
    
    public boolean isSpeedLimit(){
        return true;
    }
}
