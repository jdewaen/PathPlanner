package pathplanner;

import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.ScenarioSegmentFactory;
import pathplanner.common.Solution;
import pathplanner.common.StatisticsTracker;
import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.CPLEXSolverConfig;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfig;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristic;
import pathplanner.preprocessor.segments.CheckpointGenerator;


public class PathPlanner {
    public final CornerHeuristic cornerHeuristic;
    public final CheckpointGenerator checkpointGenerator;
    public final BoundsSolverConfig boundsConfig;
    public final CPLEXSolverConfig cplexConfig;
    public final Scenario scenario;
    public final StatisticsTracker stats = new StatisticsTracker();
    
    public PathPlanner(CornerHeuristic cornerHeuristic,
            CheckpointGenerator checkpointGenerator,
            BoundsSolverConfig boundsConfig,
            CPLEXSolverConfig cplexConfig,
            Scenario scenario){
        this.cornerHeuristic = cornerHeuristic;
        this.checkpointGenerator = checkpointGenerator;
        this.boundsConfig = boundsConfig;
        this.cplexConfig = cplexConfig;
        this.scenario = scenario;
    }
    
    public PlannerResult solve(){
        long totalTimer = stats.startTimer();
        long timer = stats.startTimer();
        PathNode prePath = cornerHeuristic.solve();
        stats.prePathTime = stats.stopTimer(timer);
        
        timer = stats.startTimer();
        List<CornerEvent> corners = cornerHeuristic.generateEvents(prePath);
        stats.cornerTime = stats.stopTimer(timer);
        
        timer = stats.startTimer();
        List<PathSegment> pathSegments = checkpointGenerator.generateFromPath(prePath, corners);
        stats.pathSegmentTime = stats.stopTimer(timer);
        
        timer = stats.startTimer();
        List<ScenarioSegmentFactory> scenariosegments = generateScenarioSegments(pathSegments);
        stats.scenSegmentTime = stats.stopTimer(timer);
        
        Solution sol;
        boolean fail = false;
        try {
            sol = solveSegments(scenariosegments);
        } catch (PlanException e) {
            System.out.println(e.parent.getMessage());
            sol = e.sol;
            fail = true;
        }
        
        stats.totalTime = stats.stopTimer(totalTimer);
        stats.score = ((double) sol.score) / cplexConfig.fps;
        PlannerResult result = new PlannerResult(sol, prePath, corners, pathSegments, stats, fail);
        return result;
    }
        
    
    public List<ScenarioSegmentFactory> generateScenarioSegments(List<PathSegment> checkpoints){
        List<ScenarioSegmentFactory> segments = new ArrayList<ScenarioSegmentFactory>();
        ScenarioSegmentFactory last = null;
        for( int i = 0; i < checkpoints.size(); i++){
            PathSegment current = checkpoints.get(i);
            ScenarioSegmentFactory segment;
            int time = (int) current.estimateTimeNeeded(scenario.vehicle, 5);
//            System.out.println("RUN " + String.valueOf(i));
//            System.out.println("TIME GUESS: " + String.valueOf(time));
            segment = new ScenarioSegmentFactory(scenario, current.end.pos, cplexConfig.fps, cplexConfig.positionTolerance, time, current);
            if( i == checkpoints.size() - 1){
                segment.isFinal = true;
                segment.positionTolerance = cplexConfig.positionToleranceFinal;
                segment.goalVel = scenario.goalVel;
            }
            if(last != null && !Double.isNaN(current.goalVel)){
//                System.out.println("Limiting speed to " + String.valueOf(current.goalVel));
                segment.maxGoalVel = current.goalVel;
            }
            last = segment;
            segments.add(segment);
        } 
        
        return segments;

    }
    
    
    public Solution solveSegments(List<ScenarioSegmentFactory> segments) throws PlanException{
        LinkedList<Solution> solutions = new LinkedList<Solution>();
//        boolean bt = false;
        
        for(int i = 0; i < segments.size(); i++){
            ScenarioSegment scen = null;
            try {
                ScenarioSegmentFactory scenFact;
                if( i < 0){
                    scenFact = segments.get(0);
                    throw new Exception();
                }else{
                    scenFact = segments.get(i);
                }
                try{
                    Solution last = solutions.getLast();
                    scenFact.startAcc = last.acc[last.score];
                    scenFact.startVel = last.vel[last.score];
                    scenFact.startPos = last.pos[last.score];
                }catch(NoSuchElementException e){
                    scenFact.startAcc = new Pos2D(0,0);
                    scenFact.startVel = scenario.startVel;
                    scenFact.startPos = scenario.startPos;
                }
                
//                System.out.println("RUN " + String.valueOf(i) + " START");
                Solution sol;
                scen = scenFact.build();
//                    if(!bt){
                        try{
//                            ScenarioSegment nextSegment;
//                            if(i + 1 < segments.size()){//TODO: return to normal
//                                nextSegment = segments.get(i + 1);
//                                nextSegment.generateActiveSet(world);
//                            }else{
//                                nextSegment = null;
//                            }
                            sol = solve(scen, null);
                        }catch(Exception e){
//                            if(i >= 25)
                                throw e;
//                            solutions.pollLast();
//                            i -= 2;
//                            System.out.println("BLOCKED: BACKTRACKING...");
//                            bt = true;
//                            continue;
                        }
//                    }else{
//                        if(i + 1 < segments.size()){
//                            sol = solve(scen, segments.get(i + 1));
//                        }else{
//                            sol = solve(scen, null);
//                        }
//                        bt = false;
//                    }
 
                addConstraintsToSol(scen, sol);
                solutions.addLast(sol);
                
            } catch (Exception e) {
//                e.printStackTrace();
                int time = 10;
                int timesteps = cplexConfig.fps * time;
                Solution empty = new Solution(time, timesteps);
                empty.highlightPoints.add(scen.startPos);
                empty.highlightPoints.add(scen.goal);
                for(int j = 0; j < timesteps; j++){
                    empty.nosol[j] = true;
                    empty.time[j] = ((double) j) / cplexConfig.fps;
                    empty.pos[j] = scen.startPos;
                    empty.vel[j] = scen.startVel;
                }
                empty.score = 10;
                addConstraintsToSol(scen, empty);
                solutions.add(empty);
                throw new PlanException(Solution.combine(solutions), e);
            } finally{
//                System.out.println("RUN " + String.valueOf(i) + " COMPLETED");
            }
        }


        Solution result = Solution.combine(solutions);

        return result;
    }
    
    private void addConstraintsToSol(ScenarioSegment scen, Solution sol){
        HashSet<Obstacle2DB> activeObs = new HashSet<Obstacle2DB>();
        
        for(ObstacleConstraint cons : scen.activeSet){
            if(cons instanceof PolygonConstraint){
                PolygonConstraint obs = (PolygonConstraint) cons;
                activeObs.add((Obstacle2DB) obs.region);
            }
        }
        
        for(int i = 0; i < sol.timeSteps; i++){
            sol.activeArea.add(scen.activeRegion);
            sol.activeObstacles[i] = activeObs;
        }
    }
    


    private Solution solve(ScenarioSegment segment, ScenarioSegment nextSegment) throws Exception{
        long geneticStart = stats.startTimer();
        segment.generateActiveSet(scenario, boundsConfig);
        long geneticDuration = stats.stopTimer(geneticStart);
        long setupStart = stats.startTimer();
        CPLEXSolver solver = new CPLEXSolver(scenario, segment, nextSegment, cplexConfig);
        solver.generateConstraints();
        long setupDuration = stats.stopTimer(setupStart);
        long solveStart = stats.startTimer();
        solver.solve();
        long solveDuration = stats.stopTimer(solveStart);
        Solution result = null;
        try {
            result = solver.getResults();
            stats.geneticTimes.add(geneticDuration);
            stats.setupTime.add(setupDuration);
            stats.solveTime.add(solveDuration);
            return result;
        } catch (IloException e) {
            throw e;
        } finally{
            solver.end();
        }

    }
    

}
