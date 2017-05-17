package pathplanner.milpplanner;


public class CPLEXSolverConfigFactory {
    public double fuzzyDelta = 0.01;
    public double absMIPgap = 1;
    public double timeLimit = 120;
    public int minSpeedPoints = 3;
    public int maxSpeedPoints = 12;
    public int maxAccPoints = 12;
    public double maxFinishAngle = 10 * Math.PI / 360;
    public int fps = 5;
    public double positionTolerance = 3;
    public double positionToleranceFinal = 0.1;
    public double minTimeLimit = 5;
    public boolean useIndicatorConstraints = true;
    public boolean ignoreVehicleSize = false;
    public boolean preventCornerCutting = true;
    public boolean useFinishLine = true;
    public boolean verbose = false;
    
    public static final CPLEXSolverConfig DEFAULT = (new CPLEXSolverConfigFactory()).build();
    
    
    
    public CPLEXSolverConfig build(){
        return new CPLEXSolverConfig(
                fuzzyDelta, 
                absMIPgap,
                timeLimit,
                minSpeedPoints,
                maxSpeedPoints,
                maxAccPoints,
                maxFinishAngle,
                fps,
                positionTolerance,
                positionToleranceFinal,
                minTimeLimit,
                useIndicatorConstraints,
                ignoreVehicleSize,
                preventCornerCutting,
                useFinishLine,
                verbose);
    }
}
