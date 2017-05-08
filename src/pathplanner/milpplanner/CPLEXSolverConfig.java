package pathplanner.milpplanner;


public class CPLEXSolverConfig {
    public final double fuzzyDelta;
    public final double MIPgap;
    public final double timeLimit;
    public final int minSpeedPoints;
    public final int maxSpeedPoints;
    public final int maxAccPoints;
    public final double maxFinishAngle;
    public final int fps;
    public final double positionTolerance;
    public final double positionToleranceFinal;
    public final double minTimeLimit;
    
    
    public CPLEXSolverConfig(
            double fuzzyDelta, 
            double MIPgap,
            double timeLimit,
            int minSpeedPoints,
            int maxSpeedPoints,
            int maxAccPoints,
            double maxFinishAngle,
            int fps,
            double positionTolerance,
            double positionToleranceFinal,
            double minTimeLimit){
        this.fuzzyDelta = fuzzyDelta;
        this.MIPgap = MIPgap;
        this.timeLimit = timeLimit;
        this.minSpeedPoints = minSpeedPoints;
        this.maxSpeedPoints = maxSpeedPoints;
        this.maxAccPoints = maxAccPoints;
        this.maxFinishAngle = maxFinishAngle;
        this.fps = fps;
        this.positionTolerance = positionTolerance;
        this.positionToleranceFinal = positionToleranceFinal;
        this.minTimeLimit = minTimeLimit;
    }
}
