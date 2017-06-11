package pathplanner;

import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Collectors;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.ScenarioSegmentFactory;
import pathplanner.common.Solution;
import pathplanner.common.StatisticsTracker;
import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.CPLEXSolverConfig;
import pathplanner.milpplanner.PolygonConstraint;


/**
 * This path planner solves the ploblem using the "Pure MILP" approach, 
 * using only a single MILP problem without preprocessing
 *
 */
public class PureMILPPathPlanner {
    public final CPLEXSolverConfig cplexConfig;
    public final Scenario scenario;
    public final double maxTime;
    public final StatisticsTracker stats = new StatisticsTracker();

    
    /**
     * @param cplexConfig The configuration for the CPLEX solver
     * @param scenario The scenario to be solved
     * @param maxTime The maximum time for the trajectory, which determines the amount
     * of time steps in the MILP problem
     */
    public PureMILPPathPlanner(
            CPLEXSolverConfig cplexConfig,
            Scenario scenario,
            double maxTime){
        this.cplexConfig = cplexConfig;
        this.scenario = scenario;
        this.maxTime = maxTime;
    }
    
    public PlannerResult solve(){
        Solution sol;
        long timer = stats.startTimer();
        boolean fail = false;
        List<ScenarioSegment> scenSegments = new ArrayList<ScenarioSegment>();
        try {            
            sol = solveSingle(scenSegments);
        } catch (Exception e) {
//            e.printStackTrace();
            sol = Solution.generateEmptySolution();
            fail = true;
        }
        sol.highlightPoints.add(scenario.startPos);
        sol.highlightPoints.add(scenario.goal);
        for(int i = 0; i < sol.timeSteps; i++){
            sol.activeObstacles[i] = new HashSet<Obstacle2DB>(scenario.world.getObstacles());
        }
        stats.solveTime.add(stats.stopTimer(timer));
        stats.totalTime = stats.stopTimer(timer);
        stats.score = ((double) sol.score) / cplexConfig.fps;
        PlannerResult result = new PlannerResult(null, sol, null, null, null, scenSegments, stats, fail);
        return result;
    }
    
  private Solution solveSingle(List<ScenarioSegment> scenSegments) throws Exception{
      ScenarioSegmentFactory segmentFact = new ScenarioSegmentFactory(scenario, scenario.goal, cplexConfig.fps, cplexConfig.positionToleranceFinal, maxTime, null);
      segmentFact.startPos = scenario.startPos;
      segmentFact.startVel = scenario.startVel;
      segmentFact.isFinal = true;
      segmentFact.goalVel = scenario.goalVel;
      ScenarioSegment segment = segmentFact.build();
      scenSegments.add(segment);
      segment.activeSet.addAll(scenario.world.getObstacles().stream().map(obs -> new PolygonConstraint(obs)).collect(Collectors.toList()));
      CPLEXSolver solver = new CPLEXSolver(scenario, segment, cplexConfig);
      solver.generateConstraints();
      solver.solve();
      Solution result = null;
      try {
          result = solver.getResults();
          return result;
      } catch (IloException e) {
//          e.printStackTrace();
          throw new Exception();
      } finally{
          solver.end();
      }
    
    }
    

}
