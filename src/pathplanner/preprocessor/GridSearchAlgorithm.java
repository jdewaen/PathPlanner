package pathplanner.preprocessor;

import java.util.LinkedList;
import java.util.Map;

import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
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
    
    public abstract LinkedList<Node> solve(double gridSize);
    public abstract LinkedList<Node> solve(double gridSize, Pos2D start);
    public abstract void solve(double gridSize, Pos2D start, Map<Pos2D, LinkedList<Node>> result);
    
    public abstract GridSearchAlgorithm buildAlgo(Scenario scenario, World2D world);
}
