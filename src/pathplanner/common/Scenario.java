package pathplanner.common;

import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.preprocessor.PathSegment;


public class Scenario {
    public final World2D world;
    public final Vehicle vehicle;
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public List<ScenarioSegment> segments;
    static final double POSITION_TOLERANCE = 3;
    static final double POSITION_TOLERANCE_FINAL = 0.1;
    static final int FPS = 5;

    public Scenario(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D startVel, Pos2D goal, Pos2D goalVel){
        if(world == null){
            throw new IllegalArgumentException("World cannot be null");
        }
        if(vehicle == null){
            throw new IllegalArgumentException("Vehicle cannot be null");
        }
        if(startPos == null){
            throw new IllegalArgumentException("StartPos cannot be null");
        }
        if(goal == null){
            throw new IllegalArgumentException("Goal cannot be null");
        }

        if(!world.isInside(startPos)){
            throw new IllegalArgumentException("StartPos is not inside world");
        }
        if(!world.isInside(goal)){
            throw new IllegalArgumentException("Goal is not inside world");
        }
        this.world = world;
        this.vehicle = vehicle;
        this.startPos = startPos;
        this.startVel = startVel;
        this.goal = goal;
        this.goalVel = goalVel;
    }


    public void  generateSegments(List<PathSegment> checkpoints) throws Exception{
        segments = new ArrayList<ScenarioSegment>();
        ScenarioSegment last = null;
        for( int i = 0; i < checkpoints.size(); i++){
            PathSegment current = checkpoints.get(i);
            ScenarioSegment segment;
            int time = (int) current.estimateTimeNeeded(vehicle, 5);
            System.out.println("RUN " + String.valueOf(i));
            System.out.println("TIME GUESS: " + String.valueOf(time));
            if( i != checkpoints.size() - 1){
                segment = new ScenarioSegment(world, vehicle, current.start.pos, null, current.end.pos, null, Double.NaN, time, time*FPS, current, vehicle.size * POSITION_TOLERANCE, false);
            }else{
                segment = new ScenarioSegment(world, vehicle, current.start.pos, null, current.end.pos, goalVel, Double.NaN, time, time*FPS, current, vehicle.size * POSITION_TOLERANCE_FINAL, true); 
            }
            if(last != null && !Double.isNaN(current.goalVel)){
                System.out.println("Limiting speed to " + String.valueOf(current.goalVel));
                segment.maxGoalVel = current.goalVel;
            }
            last = segment;
            segments.add(segment);
        } 


    }

    public Solution solve(){
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
                    lastSpeed = startVel;
                    lastPos = startPos;
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
//                             
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
                int timesteps = FPS * time;
                Solution empty = new Solution(time, timesteps);
                empty.highlightPoints.add(scen.startPos);
                empty.highlightPoints.add(scen.goal);
                for(int j = 0; j < timesteps; j++){
                    empty.nosol[j] = true;
                    empty.time[j] = ((double) j) / FPS;
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
    
//    private int getRollbackIndex(Solution sol, int finishIndex){
//        int result = finishIndex;
//        double dist = 0;
//        while( result > 0 && dist < vehicle.getAccDist()){
//            dist += sol.pos[result].distanceFrom(sol.pos[--result]);
//        }
//        
//        return result;
//    }

    private Solution solve(ScenarioSegment segment, ScenarioSegment nextSegment) throws Exception{

        segment.generateActiveSet(world);
        CPLEXSolver solver = new CPLEXSolver(this, segment, nextSegment);
        solver.generateConstraints();
        solver.solve();
        Solution result = null;
        try {
            result = solver.getResults();
            return result;
        } catch (IloException e) {
            e.printStackTrace();
            throw new Exception();
        } finally{
            solver.end();
        }

    }
}
