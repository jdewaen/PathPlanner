package pathplanner.preprocessor;

import java.util.Map;

import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.StatisticsTracker;
import pathplanner.common.World2D;


public abstract class GridSearchAlgorithm {

    public final Scenario scenario;
    public final World2D world;
    static final double SQRT2 = Math.sqrt(2);
    GridSearchAlgorithm(Scenario scenario, World2D world){
        this.scenario = scenario;
        this.world = world;
    }
    
    GridSearchAlgorithm(Scenario scenario){
        this(scenario, scenario.world);
    }
    
    GridSearchAlgorithm(){
        this(null, null);
    }
    
    public abstract PathNode solve(double gridSize, StatisticsTracker stats);
    public abstract PathNode solve(double gridSize, Pos2D start, StatisticsTracker stats);
    public abstract PathNode solve(double gridSize, Pos2D start, Pos2D goal, StatisticsTracker stats);
    
    public abstract GridSearchAlgorithm buildAlgo(Scenario scenario, World2D world);
}
