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
    public final boolean enableBacktracking;
    public final boolean verbose;
    public final int overlap;

    
    public PathPlanner(CornerHeuristic cornerHeuristic,
            CheckpointGenerator checkpointGenerator,
            BoundsSolverConfig boundsConfig,
            CPLEXSolverConfig cplexConfig,
            Scenario scenario,
            boolean enableBacktracking,
            int overlap,
            boolean verbose){
        this.cornerHeuristic = cornerHeuristic;
        this.checkpointGenerator = checkpointGenerator;
        this.boundsConfig = boundsConfig;
        this.cplexConfig = cplexConfig;
        this.scenario = scenario;
        this.enableBacktracking = enableBacktracking;
        this.overlap = overlap;
        this.verbose = verbose;
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
        List<ScenarioSegmentFactory> scenariosegmentFacts = generateScenarioSegments(pathSegments);
        stats.scenSegmentTime = stats.stopTimer(timer);
        
        Solution sol;
        boolean fail = false;
        List<ScenarioSegment> scenSegments = new ArrayList<ScenarioSegment>();
        try {
            sol = solveSegments(scenariosegmentFacts, scenSegments);
        } catch (PlanException e) {
            println(e.parent.getMessage());
            sol = e.sol;
            fail = true;
        }
        
        stats.totalTime = stats.stopTimer(totalTimer);
        stats.score = ((double) sol.score) / cplexConfig.fps;
        PlannerResult result = new PlannerResult(this, sol, prePath, corners, pathSegments, scenSegments, stats, fail);
        return result;
    }
        
    
    public List<ScenarioSegmentFactory> generateScenarioSegments(List<PathSegment> checkpoints){
        List<ScenarioSegmentFactory> segments = new ArrayList<ScenarioSegmentFactory>();
        for( int i = 0; i < checkpoints.size(); i++){
            PathSegment current = checkpoints.get(i);
            ScenarioSegmentFactory segment;
            int time = (int) current.estimateTimeNeeded(scenario.vehicle, 5, cplexConfig.timeLimitMultiplier) + overlap;
            println("RUN " + String.valueOf(i));
            println("TIME GUESS: " + String.valueOf(time));
            segment = new ScenarioSegmentFactory(scenario, current.end.pos, cplexConfig.fps, cplexConfig.positionTolerance, time, current);
            if( i == checkpoints.size() - 1){
                segment.isFinal = true;
                segment.positionTolerance = cplexConfig.positionToleranceFinal;
                segment.goalVel = scenario.goalVel;
            }
            if(!Double.isNaN(current.goalVel)){
                println("Limiting speed to " + String.valueOf(current.goalVel));
                segment.maxGoalVel = current.goalVel;
            }
            segments.add(segment);
        } 
        
        return segments;

    }
    
    
    public Solution solveSegments(List<ScenarioSegmentFactory> segmentFacts, List<ScenarioSegment> scenarioSegments) throws PlanException{
        LinkedList<Solution> solutions = new LinkedList<Solution>();
//        boolean bt = false;
        
        for(int i = 0; i < segmentFacts.size(); i++){
            ScenarioSegment scen = null;
            try {
                ScenarioSegmentFactory scenFact;
                if( i < 0){
                    scenFact = segmentFacts.get(0);
                    throw new Exception();
                }else{
                    scenFact = segmentFacts.get(i);
                }
                try{
                    Solution last = solutions.getLast();
                    if(scenario.vehicle.hasMaxJerk()) scenFact.startAcc = last.acc[last.score - overlap + 1];
                    scenFact.startVel = last.vel[last.score - overlap + 1];
                    scenFact.startPos = last.pos[last.score - overlap + 1];
                }catch(NoSuchElementException e){
                    scenFact.startAcc = new Pos2D(0,0);
                    scenFact.startVel = scenario.startVel;
                    scenFact.startPos = scenario.startPos;
                }
                
                println("RUN " + String.valueOf(i) + " START");
                Solution sol;
                scen = scenFact.build();
                scenarioSegments.add(scen);
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
//                            println("BLOCKED: BACKTRACKING...");
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
                Solution empty;
                if(solutions.isEmpty()){
                    int time = 10;
                    int timesteps = cplexConfig.fps * time;
                    empty = new Solution(time, timesteps);
                    empty.highlightPoints.add(scen.startPos);
                    empty.highlightPoints.add(scen.goal);
                    for(int j = 0; j < timesteps; j++){
                        empty.nosol[j] = true;
                        empty.time[j] = ((double) j) / cplexConfig.fps;
                        empty.pos[j] = scen.startPos;
                        empty.vel[j] = scen.startVel;
                    }
                    empty.score = 10;
                }else{
                    Solution last = solutions.getLast();
                    int timesteps = last.timeSteps - last.score + overlap - 1;
                    empty = new Solution((double) timesteps / cplexConfig.fps, timesteps);
                    empty.highlightPoints.add(scen.startPos);
                    empty.highlightPoints.add(scen.goal);
                    for(int j = 0; j < timesteps; j++){
                        empty.nosol[j] = true;
                        empty.time[j] = ((double) j) / cplexConfig.fps;
                        empty.pos[j] = last.pos[last.score - overlap + 1 + j ];
                        empty.vel[j] = last.vel[last.score - overlap + 1 + j ];
                    }
                    empty.score = timesteps - 1;
                }
                addConstraintsToSol(scen, empty);
                solutions.add(empty);
                throw new PlanException(Solution.combine(solutions, overlap), e);
            } finally{
                println("RUN " + String.valueOf(i) + " COMPLETED");
            }
        }


        Solution result = Solution.combine(solutions, overlap);

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
            sol.boundsDebugData.add(scen.boundsDebugData);
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
    
    private void println(String text){
        if(verbose){
            System.out.println(text);
        }
    }
    

}
