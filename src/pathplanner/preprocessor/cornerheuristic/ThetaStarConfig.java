package pathplanner.preprocessor.cornerheuristic;

import pathplanner.common.Scenario;


public class ThetaStarConfig implements CornerHeuristicConfig {
    public final double gridSize;
    public final double tolerance;
    public final boolean verbose;
    
    public ThetaStarConfig(double gridSize, double tolerance, boolean verbose){
        this.gridSize = gridSize;
        this.tolerance = tolerance;
        this.verbose = verbose;
    }

    @Override
    public CornerHeuristic buildHeuristic(Scenario scenario) {
        return new ThetaStar(scenario, this);
    }
    
}
