package pathplanner;

import java.util.List;

import pathplanner.common.ScenarioSegment;
import pathplanner.common.Solution;
import pathplanner.common.StatisticsTracker;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;


public class PlannerResult {
    public final Solution solution;
    public final PathNode heuristicPath;
    public final List<CornerEvent> cornerEvents;
    public final List<PathSegment> pathSegments;
    public final List<ScenarioSegment> scenarioSegments;
    public final StatisticsTracker stats;
    public final boolean failed;
    public final PathPlanner planner;
    
//    public PlannerResult(Solution solution, 
//            PathNode heuristicPath, 
//            List<CornerEvent> cornerEvents, 
//            List<PathSegment> pathSegments,
//            StatisticsTracker stats){
//        this(solution, heuristicPath, cornerEvents, pathSegments, stats, false);
//    }
    
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

}
