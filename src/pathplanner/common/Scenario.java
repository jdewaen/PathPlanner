package pathplanner.common;

import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.RectConstraint;
import pathplanner.milpplanner.RegularLine;
import pathplanner.milpplanner.VerticalLine;
import pathplanner.preprocessor.PathSegment;


public class Scenario {
    public final World2D world;
    public final Vehicle vehicle;
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public List<ScenarioSegment> segments;
    static final double POSITION_TOLERANCE = 1;
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
//        int time = 40;
        ScenarioSegment last = null;
        for( int i = 0; i < checkpoints.size(); i++){
            PathSegment current = checkpoints.get(i);
            ScenarioSegment segment;
            int time = (int) current.estimateTimeNeeded(vehicle, 20);
            System.out.println("RUN " + String.valueOf(i));
            System.out.println("TIME GUESS: " + String.valueOf(time));
            if( i != checkpoints.size() - 1){
                segment = new ScenarioSegment(world, vehicle, current.start.pos, null, current.end.pos, null, Double.NaN, time, time*FPS, current, POSITION_TOLERANCE);
            }else{
                segment = new ScenarioSegment(world, vehicle, current.start.pos, null, current.end.pos, goalVel, Double.NaN, time, time*FPS, current, POSITION_TOLERANCE_FINAL); 
            }
            if(last != null && !Double.isNaN(current.goalVel)){
                System.out.println("Limiting speed to " + String.valueOf(current.goalVel));
                segment.maxGoalVel = current.goalVel;
//                last.maxGoalVel = current.goalVel;
            }
            last = segment;
            segments.add(segment);
        } 

        for(ScenarioSegment scen : segments) scen.generateActiveSet(world);

    }

    public Solution solve(){
        List<Solution> solutions = new ArrayList<Solution>();
        Pos2D lastSpeed = startVel;
        Pos2D lastPos = startPos;
        int runNum = 0;
        for(ScenarioSegment scen: segments){
            try {
                System.out.println("RUN " + String.valueOf(runNum) + " START");
                scen.startVel = lastSpeed;
                scen.startPos = lastPos;
                Solution sol = solve(scen);
                addConstraintsToSol(scen, sol);
//                int index = getRollbackIndex(sol, sol.score);
                lastSpeed = sol.vel[sol.score];
                lastPos = sol.pos[sol.score];
                solutions.add(sol);
            } catch (Exception e) {
                e.printStackTrace();
                int time = 10;
                int timesteps = FPS * time;
                Solution empty = new Solution(time, timesteps);
                empty.highlightPoints.add(scen.startPos);
                empty.highlightPoints.add(scen.goal);
                for(int i = 0; i < timesteps; i++){
                    empty.nosol[i] = true;
                    empty.time[i] = ((double) i) / FPS;
                }
                empty.score = 10;
                addConstraintsToSol(scen, empty);
                solutions.add(empty);
                break;
            } finally{
                System.out.println("RUN " + String.valueOf(runNum) + " COMPLETED");
                runNum++;
            }
        }


        Solution result = Solution.combine(solutions);

        return result;
    }
    
    private void addConstraintsToSol(ScenarioSegment scen, Solution sol){
        double above = 0;
        double below = Double.MAX_VALUE;
        double left = Double.MAX_VALUE;
        double right = 0;
        HashSet<Obstacle2D> activeObs = new HashSet<Obstacle2D>();
        
        for(ObstacleConstraint cons : scen.activeSet){
            if(cons instanceof RegularLine){
                RegularLine line = (RegularLine) cons;
                if(line.above && line.b > above) above = line.b;
                else if(!line.above && line.b < below) below = line.b;
            }
            if(cons instanceof VerticalLine){
                VerticalLine line = (VerticalLine) cons;
                if(line.left && line.x < left) left = line.x;
                else if(!line.left && line.x > right) right = line.x;
            }
            if(cons instanceof RectConstraint){
                RectConstraint obs = (RectConstraint) cons;
                activeObs.add((Obstacle2D) obs.region);
            }
        }
        
//        System.out.println("ABOVE " + String.valueOf(above));
//        System.out.println("BELOW " + String.valueOf(below));
//        System.out.println("LEFT  " + String.valueOf(left));
//        System.out.println("RIGHT " + String.valueOf(right));
        
        Region2D activeRegion = new Region2D(new Pos2D(right, above), new Pos2D(left, below)){};
        for(int i = 0; i < sol.timeSteps; i++){
            sol.activeArea[i] = activeRegion;
            sol.activeObstacles[i] = activeObs;
        }
    }
    
    private int getRollbackIndex(Solution sol, int finishIndex){
        int result = finishIndex;
        double dist = 0;
        while( result > 0 && dist < vehicle.getAccDist()){
            dist += sol.pos[result].distanceFrom(sol.pos[--result]);
        }
        
        return result;
    }

    private Solution solve(ScenarioSegment segment) throws Exception{

        CPLEXSolver solver = new CPLEXSolver(this, segment);
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
