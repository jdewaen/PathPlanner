package pathplanner.preprocessor.boundssolver;


public class BoundsSolverConfigFactory {
    public int popSize = 10;
    public int maxGens = 25;
    public double pathLengthMultiplier = 2;
    
    public double mutationProb = 1;
    public int maxNudgeDistance = 5;
    public int minPoints = 4;
    public int maxPoints = 12;
    public double addPointProb = 0.1;
    public double removePointProb = 0.1;
    public int maxAttempts = 15;

    public static final BoundsSolverConfig DEFAULT = (new BoundsSolverConfigFactory()).build();
    
    public BoundsSolverConfig build(){
        return new BoundsSolverConfig(
                popSize, 
                maxGens, 
                pathLengthMultiplier, 
                mutationProb, 
                maxNudgeDistance, 
                minPoints, 
                maxPoints, 
                addPointProb, 
                removePointProb,
                maxAttempts);
    }
}
