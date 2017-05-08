package pathplanner.preprocessor.cornerheuristic;

import pathplanner.common.Scenario;


public class AStarConfig implements CornerHeuristicConfig {
    public final double gridSize;
    public final double tolerance;
    public static final AStarConfig DEFAULT = new AStarConfig(2, 2);
    
    public AStarConfig(double gridSize, double tolerance){
        this.gridSize = gridSize;
        this.tolerance = tolerance;
    }

    @Override
    public CornerHeuristic buildHeuristic(Scenario scenario) {
        return new FixedAStar(scenario, this);
    }
    
}
