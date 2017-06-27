package pathplanner;

import java.io.Serializable;
import java.util.List;

import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.Solution;
import pathplanner.common.StatisticsTracker;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;


public class PlannerResult implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -8279672248945164305L;
    public final Solution solution;
    public final PathNode heuristicPath;
    public final List<CornerEvent> cornerEvents;
    public final List<PathSegment> pathSegments;
    public final List<ScenarioSegment> scenarioSegments;
    public final StatisticsTracker stats;
    public final boolean failed;
    public final PathPlanner planner;
    
    public PlannerResult(
            PathPlanner planner,
            Solution solution, 
            PathNode heuristicPath, 
            List<CornerEvent> cornerEvents, 
            List<PathSegment> pathSegments,
            List<ScenarioSegment> scenarioSegments,
            StatisticsTracker stats,
            boolean failed){
        this.planner = planner;
        this.solution = solution;
        this.heuristicPath = heuristicPath;
        this.cornerEvents = cornerEvents;
        this.pathSegments = pathSegments;
        this.scenarioSegments = scenarioSegments;
        this.stats = stats;
        this.failed = failed;
    }
    
    public Scenario getScenario(){
        return planner.scenario;
    }

}
