package pathplanner;

import pathplanner.common.Scenario;
import pathplanner.milpplanner.CPLEXSolverConfig;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfig;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfigFactory;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristicConfig;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfig;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfigFactory;
import pathplanner.preprocessor.segments.PathSegmentGenerator;
import pathplanner.preprocessor.segments.PathSegmentGeneratorConfig;
import pathplanner.preprocessor.segments.PathSegmentGeneratorConfigFactory;


/**
 * A factory class for the PathPlanner class
 *
 */
public class PathPlannerFactory {
    public CornerHeuristicConfig cornerConfig = ThetaStarConfigFactory.DEFAULT;
    public PathSegmentGeneratorConfig segmentConfig = PathSegmentGeneratorConfigFactory.DEFAULT;
    public BoundsSolverConfig boundsConfig = BoundsSolverConfigFactory.DEFAULT;
    public CPLEXSolverConfig cplexConfig = CPLEXSolverConfigFactory.DEFAULT;
    public boolean enableBacktracking = false;
    public boolean useStopPoints = true;
    public int overlap = 1;
    public boolean verbose = false;
    
    
    public PathPlanner build(Scenario scenario){
        return new PathPlanner(
                cornerConfig.buildHeuristic(scenario), 
                new PathSegmentGenerator(scenario, segmentConfig), 
                boundsConfig,
                cplexConfig,
                scenario,
                enableBacktracking,
                useStopPoints,
                overlap,
                verbose);
    }

}
