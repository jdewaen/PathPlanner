package pathplanner.preprocessor.cornerheuristic;

import pathplanner.common.Scenario;


public class ThetaStarConfig implements CornerHeuristicConfig {
    public final double gridSize;
    public final double tolerance;
    public static final ThetaStarConfig DEFAULT = new ThetaStarConfig(2, 2);
    
    public ThetaStarConfig(double gridSize, double tolerance){
        this.gridSize = gridSize;
        this.tolerance = tolerance;
    }

    @Override
    public CornerHeuristic buildHeuristic(Scenario scenario) {
        return new ThetaStar(scenario, this);
    }
    
}
