package pathplanner.preprocessor.boundssolver;

import java.io.Serializable;


public class BoundsSolverConfig implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -1102115161068954478L;
    public final double convexGrowMultiplier;
    public final int initRegionVertices;
    public final boolean useStopPoints;
    
    public final int popSize;
    public final int maxGens;
    public final double pathLengthMultiplier;
    
    public final double mutationProb;
    public final int maxNudgeDistance;
    public final int minPoints;
    public final int maxPoints;
    public final double addPointProb;
    public final double removePointProb;
    public final int maxAttempts;
    public final boolean verbose;

    
    public BoundsSolverConfig(
            double convexGrowMultiplier,
            int initRegionVertices,
            boolean useStopPoints,
            int popSize,
            int maxGens,
            double pathLengthMultiplier,
            double mutationProb,
            int maxNudgeDistance,
            int minPoints,
            int maxPoints,
            double addPointProb,
            double removePointProb,
            int maxAttempts,
            boolean verbose
            ){
        this.convexGrowMultiplier = convexGrowMultiplier;
        this.initRegionVertices = initRegionVertices;
        this.useStopPoints = useStopPoints;
        this.popSize = popSize;
        this.maxGens = maxGens;
        this.pathLengthMultiplier = pathLengthMultiplier;
        this.mutationProb = mutationProb;
        this.maxNudgeDistance = maxNudgeDistance;
        this.minPoints = minPoints;
        this.maxPoints = maxPoints;
        this.addPointProb = addPointProb;
        this.removePointProb = removePointProb;
        this.maxAttempts = maxAttempts;
        this.verbose = verbose;
    }
}
