package pathplanner.preprocessor.cornerheuristic;

import java.io.Serializable;

import pathplanner.common.Scenario;


public class ThetaStarConfig implements CornerHeuristicConfig, Serializable {
    /**
     * 
     */
    private static final long serialVersionUID = -5701429045067930369L;
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
