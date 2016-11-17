package pathplanner;

import java.util.ArrayList;
import java.util.List;

import javax.management.RuntimeErrorException;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {
    
//    public static Scenario2D generateBenchmarkScenario(Vehicle vehicle){
//        World2D world = new World2D(new Pos2D(40, 20));
//      world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13), 0, 10));
//      
//      world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20), 0, 15));
//      
//      world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13), 6, 18));
//      
////      world.addRegion(new CheckPoint2D(new Pos2D(10, 13), new Pos2D(12, 20), 15));
//
////      world.addRegion(new CheckPoint2D(new Pos2D(22, 5), new Pos2D(26, 12), 16));
//
//      
//      Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 1), new Pos2D(38, 10), 19, 200);
//
//      return scen;
//    }
    
//    public static Scenario2D generateBenchmarkScenario2(Vehicle vehicle){
//        World2D world = new World2D(new Pos2D(40, 20));
//      world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13), 0, 11));
//      
//      world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20), 3, 11));
//      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(8, 20), 11, 13));
////      world.addRegion(new CheckPoint2D(new Pos2D(6, 0), new Pos2D(8, 5), 9));
//
//      
//      world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13), 7, 13));
//      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(12, 20), 13, 16));
////
////      
//      world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20), 9, 16));
//      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(16, 20), 16, 19));
////      world.addRegion(new CheckPoint2D(new Pos2D(14, 0), new Pos2D(16, 5), 15));
//
////
////      
//      world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13), 14, 19));
//      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(20, 20), 19, 21));
//     
//      
//      
////      world.addRegion(new CheckPoint2D(new Pos2D(10, 13), new Pos2D(12, 20), 15));
//
//
//      
//      Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 1), new Pos2D(38, 10), 24, 100);
//
//      return scen;
//    }
//    
//    public static Scenario2D checkPointTestScenario(Vehicle vehicle){
//        World2D world = new World2D(new Pos2D(40, 20));
//        
//        world.addRegion(new CheckPoint2D(new Pos2D(20, 16), new Pos2D(24, 18), 5));
////        world.addRegion(new CheckPoint2D(new Pos2D(2, 4), new Pos2D(4, 6), 5));
//      
//      Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 5), new Pos2D(38, 5), 19, 200);
//
//      return scen;
//    }
   
    
    
    public static World2D generateBenchmarkWorld(){
        World2D world = new World2D(new Pos2D(40, 20));
      world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13)));
//        world.addRegion(new Sphere2D(new Pos2D(3, 2), 1));
//        world.addRegion(new Sphere2D(new Pos2D(3, 4), 1));
//        world.addRegion(new Sphere2D(new Pos2D(3, 6), 1));
//        world.addRegion(new Sphere2D(new Pos2D(3, 8), 1));
//        world.addRegion(new Sphere2D(new Pos2D(3, 10), 1));
//        world.addRegion(new Sphere2D(new Pos2D(3, 12), 1));

  //    
      world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20)));
  //
      world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13)));
  //  
      world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20)));
  //
      world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13)));
      
      
        return world;
    }
    
//    public static World2D generateTestWorld(){
//        World2D world = new World2D(new Pos2D(20, 20));
//        
//        world.addRegion(new Sphere2D(new Pos2D(10, 10), 5));
//        
//        return world;
//
//    }
    
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
      
      Region2D obs0 = world.getRegions().get(0);
      Region2D obs1 = world.getRegions().get(1);
      Region2D obs2 = world.getRegions().get(2);
      Region2D obs3 = world.getRegions().get(3);
      Region2D obs4 = world.getRegions().get(4);

      
      try {
        scenarios.get(0).activeSet.add(Line.fromRegion(obs0, checkpoints.get(0), false));
        
        scenarios.get(1).activeSet.add(RectConstraint.fromRegion(obs0));
        scenarios.get(1).activeSet.add(Line.fromRegion(obs1, checkpoints.get(1), false));

        
        scenarios.get(2).activeSet.add(Line.fromRegion(obs0, checkpoints.get(2), false));
        scenarios.get(2).activeSet.add(RectConstraint.fromRegion(obs1));
        scenarios.get(2).activeSet.add(Line.fromRegion(obs2, checkpoints.get(2), false));
        
        scenarios.get(3).activeSet.add(Line.fromRegion(obs1, checkpoints.get(3), false));
        scenarios.get(3).activeSet.add(RectConstraint.fromRegion(obs2));
        scenarios.get(3).activeSet.add(Line.fromRegion(obs3, checkpoints.get(3), false));
        
        scenarios.get(4).activeSet.add(Line.fromRegion(obs2, checkpoints.get(4), false));
        scenarios.get(4).activeSet.add(RectConstraint.fromRegion(obs3));
        scenarios.get(4).activeSet.add(Line.fromRegion(obs4, checkpoints.get(4), false));
        
        scenarios.get(5).activeSet.add(Line.fromRegion(obs3, checkpoints.get(5), false));
        scenarios.get(5).activeSet.add(RectConstraint.fromRegion(obs4));


    } catch (Exception e) {
        e.printStackTrace();
    }

    return scenarios;
    }

    public static void main(String[] args) {
        
        Vehicle vehicle = new Vehicle(3, 5);
        
        World2D world = generateBenchmarkWorld();
//        World2D world = generateTestWorld();
        
        List<Scenario2D> scenarios = sphereBenchmarkEquivalent(vehicle, world);
//        List<Scenario2D> scenarios = sphereTest(vehicle, world);
        
        List<Solution> solutions = new ArrayList<Solution>();
        
        
        long startTime = System.currentTimeMillis();

        


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
