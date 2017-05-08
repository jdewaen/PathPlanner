package pathplanner;

import java.util.List;

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
    public final StatisticsTracker stats;
    
    public PlannerResult(Solution solution, 
            PathNode heuristicPath, 
            List<CornerEvent> cornerEvents, 
            List<PathSegment> pathSegments,
            StatisticsTracker stats){
        this.solution = solution;
        this.heuristicPath = heuristicPath;
        this.cornerEvents = cornerEvents;
        this.pathSegments = pathSegments;
        this.stats = stats;
    }

}
