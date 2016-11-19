package pathplanner.common;

import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.List;

import pathplanner.milpplanner.CPLEXSolver;


public class Scenario {
    public final World2D world;
    public final Vehicle vehicle;
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public List<ScenarioSegment> segments;

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


    public void  generateSegments(List<Pos2D> checkpoints) throws Exception{
        segments = new ArrayList<ScenarioSegment>();
        for( int i = 1; i < checkpoints.size(); i++){
            if( i != checkpoints.size() - 1){
                segments.add(new ScenarioSegment(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), null, 10, 100));
            }else{
                segments.add(new ScenarioSegment(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), goalVel, 10, 100)); 
            }    
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
                scen.startVel = lastSpeed;
                scen.startPos = lastPos;
                Solution sol = solve(scen);
                lastSpeed = sol.vel[sol.score];
                lastPos = sol.pos[sol.score];
                solutions.add(sol);
            } catch (Exception e) {
                e.printStackTrace();
                Solution empty = new Solution(0, 0);
                empty.highlightPoints.add(scen.startPos);
                empty.highlightPoints.add(scen.goal);
                solutions.add(empty);
                break;
            } finally{
                System.out.println("RUN " + String.valueOf(runNum));
                runNum++;
            }
        }


        Solution result = Solution.combine(solutions);

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
