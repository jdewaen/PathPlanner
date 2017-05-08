package pathplanner.preprocessor.cornerheuristic;

import pathplanner.common.Scenario;


public interface CornerHeuristicConfig {    
    public CornerHeuristic buildHeuristic(Scenario scenario);
}
