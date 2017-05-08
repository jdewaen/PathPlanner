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
import pathplanner.common.Solution;
import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristic;
import pathplanner.preprocessor.segments.CheckpointGenerator;


public class PathPlanner {
    public final CornerHeuristic cornerHeuristic;
    public final CheckpointGenerator checkpointGenerator;
    public final Scenario scenario;
    
    public PathPlanner(CornerHeuristic cornerHeuristic, CheckpointGenerator checkpointGenerator, Scenario scenario){
        this.cornerHeuristic = cornerHeuristic;
        this.checkpointGenerator = checkpointGenerator;
        this.scenario = scenario;
    }
    
    public PlannerResult solve(){
        PathNode prePath = cornerHeuristic.solve();
        List<CornerEvent> corners = cornerHeuristic.generateEvents(prePath);
        List<PathSegment> pathSegments = checkpointGenerator.generateFromPath(prePath, corners);
        List<ScenarioSegment> scenariosegments = generateScenarioSegments(pathSegments);
        Solution sol = solveSegments(scenariosegments);
        PlannerResult result = new PlannerResult(sol, prePath, corners, pathSegments);
        return result;
    }
    
    
    public List<ScenarioSegment> generateScenarioSegments(List<PathSegment> checkpoints){
        List<ScenarioSegment> segments = new ArrayList<ScenarioSegment>();
        ScenarioSegment last = null;
        for( int i = 0; i < checkpoints.size(); i++){
            PathSegment current = checkpoints.get(i);
            ScenarioSegment segment;
            int time = (int) current.estimateTimeNeeded(scenario.vehicle, 5);
            System.out.println("RUN " + String.valueOf(i));
            System.out.println("TIME GUESS: " + String.valueOf(time));
            if( i != checkpoints.size() - 1){
                segment = new ScenarioSegment(scenario.world, scenario.vehicle, current.start.pos, null, current.end.pos, null, Double.NaN, time, time*scenario.FPS, current, scenario.vehicle.size * scenario.POSITION_TOLERANCE, false);
            }else{
                segment = new ScenarioSegment(scenario.world, scenario.vehicle, current.start.pos, null, current.end.pos, scenario.goalVel, Double.NaN, time, time*scenario.FPS, current, scenario.vehicle.size * scenario.POSITION_TOLERANCE_FINAL, true); 
            }
            if(last != null && !Double.isNaN(current.goalVel)){
                System.out.println("Limiting speed to " + String.valueOf(current.goalVel));
                segment.maxGoalVel = current.goalVel;
            }
            last = segment;
            segments.add(segment);
        } 
        
        return segments;

    }
    
    
    public Solution solveSegments(List<ScenarioSegment> segments){
        LinkedList<Solution> solutions = new LinkedList<Solution>();
        Pos2D lastAcc;
        Pos2D lastSpeed;
        Pos2D lastPos;
        boolean bt = false;
        
        for(int i = 0; i < segments.size(); i++){
            ScenarioSegment scen = null;
            try {
                if( i < 0){
                    scen = segments.get(0);
                    throw new Exception();
                }else{
                    scen = segments.get(i);
                }
                try{
                    Solution last = solutions.getLast();
                    lastAcc = last.acc[last.score];
                    lastSpeed = last.vel[last.score];
                    lastPos = last.pos[last.score];
                }catch(NoSuchElementException e){
                    lastAcc = new Pos2D(0,0);
                    lastSpeed = scenario.startVel;
                    lastPos = scenario.startPos;
                }
                
                System.out.println("RUN " + String.valueOf(i) + " START");
                scen.startAcc = lastAcc;
                scen.startVel = lastSpeed;
                scen.startPos = lastPos;
                Solution sol;
                    if(!bt){
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
//                                throw e;
                            solutions.pollLast();
                            i -= 2;
                            System.out.println("BLOCKED: BACKTRACKING...");
                            bt = true;
                            continue;
                        }
                    }else{
                        if(i + 1 < segments.size()){
                            sol = solve(scen, segments.get(i + 1));
                        }else{
                            sol = solve(scen, null);
                        }
                        bt = false;
                    }
 
                addConstraintsToSol(scen, sol);
                solutions.addLast(sol);
                
            } catch (Exception e) {
                e.printStackTrace();
                int time = 10;
                int timesteps = scenario.FPS * time;
                Solution empty = new Solution(time, timesteps);
                empty.highlightPoints.add(scen.startPos);
                empty.highlightPoints.add(scen.goal);
                for(int j = 0; j < timesteps; j++){
                    empty.nosol[j] = true;
                    empty.time[j] = ((double) j) / scenario.FPS;
                    empty.pos[j] = scen.startPos;
                    empty.vel[j] = scen.startVel;
                }
                empty.score = 10;
                addConstraintsToSol(scen, empty);
                solutions.add(empty);
                break;
            } finally{
                System.out.println("RUN " + String.valueOf(i) + " COMPLETED");
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
//        long geneticStart = stats.startTimer();
        segment.generateActiveSet(scenario.world);
//        long geneticDuration = stats.stopTimer(geneticStart);
//        long setupStart = stats.startTimer();
        CPLEXSolver solver = new CPLEXSolver(scenario, segment, nextSegment);
        solver.generateConstraints();
//        long setupDuration = stats.stopTimer(setupStart);
//        long solveStart = stats.startTimer();
        solver.solve();
//        long solveDuration = stats.stopTimer(solveStart);
        Solution result = null;
        try {
            result = solver.getResults();
//            stats.geneticTimes.add(geneticDuration);
//            stats.setupTime.add(setupDuration);
//            stats.solveTime.add(solveDuration);
            return result;
        } catch (IloException e) {
            e.printStackTrace();
            throw new Exception();
        } finally{
            solver.end();
        }

    }
    

}
