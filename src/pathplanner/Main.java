package pathplanner;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {
    
    public static Scenario2D generateBenchmarkScenario(Vehicle vehicle){
        World2D world = new World2D(new Pos2D(40, 20));
      world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13), 0, 10));
      
      world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20), 0, 15));
      
      world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13), 6, 18));
      
//      world.addRegion(new CheckPoint2D(new Pos2D(10, 13), new Pos2D(12, 20), 15));

//      world.addRegion(new CheckPoint2D(new Pos2D(22, 5), new Pos2D(26, 12), 16));

      
      Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 1), new Pos2D(38, 10), 19, 200);

      return scen;
    }
    
    public static Scenario2D generateBenchmarkScenario2(Vehicle vehicle){
        World2D world = new World2D(new Pos2D(40, 20));
      world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13), 0, 11));
      
      world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20), 3, 11));
      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(8, 20), 11, 13));
//      world.addRegion(new CheckPoint2D(new Pos2D(6, 0), new Pos2D(8, 5), 9));

      
      world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13), 7, 13));
      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(12, 20), 13, 16));
//
//      
      world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20), 9, 16));
      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(16, 20), 16, 19));
//      world.addRegion(new CheckPoint2D(new Pos2D(14, 0), new Pos2D(16, 5), 15));

//
//      
      world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13), 14, 19));
      world.addRegion(new Obstacle2D(new Pos2D(0, 0), new Pos2D(20, 20), 19, 21));
     
      
      
//      world.addRegion(new CheckPoint2D(new Pos2D(10, 13), new Pos2D(12, 20), 15));


      
      Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 1), new Pos2D(38, 10), 24, 100);

      return scen;
    }
    
    public static Scenario2D checkPointTestScenario(Vehicle vehicle){
        World2D world = new World2D(new Pos2D(40, 20));
        
        world.addRegion(new CheckPoint2D(new Pos2D(20, 16), new Pos2D(24, 18), 5));
//        world.addRegion(new CheckPoint2D(new Pos2D(2, 4), new Pos2D(4, 6), 5));
      
      Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 5), new Pos2D(38, 5), 19, 200);

      return scen;
    }
   

    public static void main(String[] args) {
        
        Vehicle vehicle = new Vehicle(3, 5);
        
//      world.addRegion(new SpeedLimitRegion2D(new Pos2D(0, 0), new Pos2D(40, 20), 1));

        
        Scenario2D scen = generateBenchmarkScenario2(vehicle);
//        Scenario2D scen = checkPointTestScenario(vehicle);

        
        CPLEXSolver solver = new CPLEXSolver(scen);
        solver.generateConstraints();
        long startTime = System.currentTimeMillis();
        solver.solve();
        long endTime   = System.currentTimeMillis();
        double totalTime = endTime - startTime;
        totalTime /= 1000;
        Solution result = null;
        try {
            result = solver.getResults();
            ResultWindow test = new ResultWindow(result, scen, totalTime);
            test.setVisible(true);
        } catch (IloException e) {
            e.printStackTrace();
        }
        solver.end();
        
    }

}
