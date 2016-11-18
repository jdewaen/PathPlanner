package pathplanner;

import java.util.ArrayList;
import java.util.List;

import javax.management.RuntimeErrorException;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {
       
    public static World2D generateBenchmarkWorld(){
        World2D world = new World2D(new Pos2D(40, 20));
      world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13)));
      world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20)));
      world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13)));
      world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20)));
      world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13)));
        return world;
    }
    
    
    public static List<Scenario2D> sphereTest(Vehicle vehicle, World2D world){
        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        
        checkpoints.add(new Pos2D(1, 10.1));        
        checkpoints.add(new Pos2D(5, 14.5));
        checkpoints.add(new Pos2D(10, 18));
        checkpoints.add(new Pos2D(15, 16));
        checkpoints.add(new Pos2D(19, 12));
        
        
        List<Scenario2D> scenarios = new ArrayList<Scenario2D>();

        for( int i = 1; i < checkpoints.size(); i++){
            if( i != checkpoints.size() - 1){
                scenarios.add(new Scenario2D(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), null, 10, 100));
            }else{
                scenarios.add(new Scenario2D(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), new Pos2D(0, 0), 10, 100)); 
            }
            
        }
        
        return scenarios;
    }
    
    public static List<Scenario2D> sphereBenchmarkEquivalent(Vehicle vehicle, World2D world){

   
      List<Pos2D> checkpoints = new ArrayList<Pos2D>();
      checkpoints.add(new Pos2D(1, 1));
      checkpoints.add(new Pos2D(1, 9));
      checkpoints.add(new Pos2D(5, 9));
      checkpoints.add(new Pos2D(9, 8));
      checkpoints.add(new Pos2D(13, 8));
      checkpoints.add(new Pos2D(17, 8));
      checkpoints.add(new Pos2D(38, 10));


      List<Scenario2D> scenarios = new ArrayList<Scenario2D>();

      for( int i = 1; i < checkpoints.size(); i++){
          if( i != checkpoints.size() - 1){
              scenarios.add(new Scenario2D(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), null, 10, 100));
          }else{
              scenarios.add(new Scenario2D(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), new Pos2D(0, 0), 10, 100)); 
          }
          
      } 

      
      try {
          
          for(Scenario2D scen : scenarios){
              scen.generateActiveSet();
          }


    } catch (Exception e) {
        e.printStackTrace();
    }

    return scenarios;
    }

    public static void main(String[] args) {
        
        long startTime = System.currentTimeMillis();

        
        Vehicle vehicle = new Vehicle(3, 5);        
        World2D world = generateBenchmarkWorld();
        List<Scenario2D> scenarios = sphereBenchmarkEquivalent(vehicle, world);
        
        List<Solution> solutions = new ArrayList<Solution>();
            Pos2D lastSpeed = new Pos2D(0, 0);
            Pos2D lastPos = scenarios.get(0).startPos;
            int runNum = 0;
            for(Scenario2D scen: scenarios){
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
                } finally{
                    System.out.println("RUN " + String.valueOf(runNum));
                    runNum++;
                }
            }

        
        long endTime   = System.currentTimeMillis();
        double totalTime = endTime - startTime;
        totalTime /= 1000;
        Solution result = Solution.combine(solutions);
        ResultWindow test = new ResultWindow(result, world, totalTime);
        test.setVisible(true);
        
    }
    
    private static Solution solve(Scenario2D scen) throws Exception{
        
        CPLEXSolver solver = new CPLEXSolver(scen);
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
