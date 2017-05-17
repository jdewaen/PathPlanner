package pathplanner;

import pathplanner.common.Scenario;
import pathplanner.milpplanner.CPLEXSolverConfig;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfig;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfigFactory;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristicConfig;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfig;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfigFactory;
import pathplanner.preprocessor.segments.CheckpointGenerator;
import pathplanner.preprocessor.segments.SegmentGeneratorConfig;
import pathplanner.preprocessor.segments.SegmentGeneratorConfigFactory;


public class PathPlannerFactory {
    public CornerHeuristicConfig cornerConfig = ThetaStarConfigFactory.DEFAULT;
    public SegmentGeneratorConfig segmentConfig = SegmentGeneratorConfigFactory.DEFAULT;
    public BoundsSolverConfig boundsConfig = BoundsSolverConfigFactory.DEFAULT;
    public CPLEXSolverConfig cplexConfig = CPLEXSolverConfigFactory.DEFAULT;
    public boolean enableBacktracking = false;
    public int overlap = 1;
    public boolean verbose = false;
    
    
    public PathPlanner build(Scenario scenario){
        return new PathPlanner(
                cornerConfig.buildHeuristic(scenario), 
                new CheckpointGenerator(scenario, segmentConfig), 
                boundsConfig,
                cplexConfig,
                scenario,
                enableBacktracking,
                overlap,
                verbose);
    }

}
