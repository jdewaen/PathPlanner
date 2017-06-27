package pathplanner.milpplanner;

import java.io.Serializable;


public class CPLEXSolverConfig implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -1015935578373543152L;
    public final double fuzzyDelta;
    public final double absMIPgap;
    public final double timeLimit;
    public final int minSpeedPoints;
    public final int maxSpeedPoints;
    public final int maxAccPoints;
    public final double maxFinishAngle;
    public final int fps;
    public final double positionTolerance;
    public final double positionToleranceFinal;
    public final double minTimeLimit;
    public final double timeLimitMultiplier;
    
    public final boolean useIndicatorConstraints;
    public final boolean ignoreVehicleSize;
    public final boolean preventCornerCutting;
    public final boolean useFinishLine;
    public final boolean verbose;    
    
    public CPLEXSolverConfig(
            double fuzzyDelta, 
            double absMIPgap,
            double timeLimit,
            int minSpeedPoints,
            int maxSpeedPoints,
            int maxAccPoints,
            double maxFinishAngle,
            int fps,
            double positionTolerance,
            double positionToleranceFinal,
            double minTimeLimit,
            double timeLimitMultiplier,
            boolean useIndicatorConstraints,
            boolean ignoreVehicleSize,
            boolean preventCornerCutting,
            boolean useFinishLine,
            boolean verbose){
        this.fuzzyDelta = fuzzyDelta;
        this.absMIPgap = absMIPgap;
        this.timeLimit = timeLimit;
        this.minSpeedPoints = minSpeedPoints;
        this.maxSpeedPoints = maxSpeedPoints;
        this.maxAccPoints = maxAccPoints;
        this.maxFinishAngle = maxFinishAngle;
        this.fps = fps;
        this.positionTolerance = positionTolerance;
        this.positionToleranceFinal = positionToleranceFinal;
        this.minTimeLimit = minTimeLimit;
        this.timeLimitMultiplier = timeLimitMultiplier;
        this.useIndicatorConstraints = useIndicatorConstraints;
        this.ignoreVehicleSize = ignoreVehicleSize;
        this.preventCornerCutting = preventCornerCutting;
        this.useFinishLine = useFinishLine;
        this.verbose = verbose;
    }
}
