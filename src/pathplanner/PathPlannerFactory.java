package pathplanner;

import pathplanner.common.Scenario;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristicConfig;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfig;
import pathplanner.preprocessor.segments.CheckpointGenerator;
import pathplanner.preprocessor.segments.SegmentGeneratorConfig;


public class PathPlannerFactory {
    CornerHeuristicConfig cornerConfig = ThetaStarConfig.DEFAULT;
    SegmentGeneratorConfig segmentConfig = SegmentGeneratorConfig.DEFAULT;
    
    
    
    public PathPlanner build(Scenario scenario){
        return new PathPlanner(cornerConfig.buildHeuristic(scenario), new CheckpointGenerator(scenario, segmentConfig), scenario);
    }

}
