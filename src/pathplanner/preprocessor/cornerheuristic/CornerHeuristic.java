package pathplanner.preprocessor.cornerheuristic;

import java.util.List;

import pathplanner.common.Vector2D;
import pathplanner.common.Scenario;
import pathplanner.common.World2D;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;


public abstract class CornerHeuristic {

    public final Scenario scenario;
    public final World2D world;
    static final double SQRT2 = Math.sqrt(2);
    CornerHeuristic(Scenario scenario, World2D world){
        this.scenario = scenario;
        this.world = world;
    }
    
    CornerHeuristic(Scenario scenario){
        this(scenario, scenario.world);
    }
    
    CornerHeuristic(){
        this(null, null);
    }
    
    public abstract PathNode solve();
    public abstract PathNode solve(Vector2D start);
    public abstract PathNode solve(Vector2D start, Vector2D goal);
    
    public abstract List<CornerEvent> generateEvents(PathNode path);
    }
