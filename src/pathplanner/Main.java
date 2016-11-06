package pathplanner;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {

    public static void main(String[] args) {
        

        World2D world = new World2D(new Pos2D(40, 20));
//        world.addObstacle(new Obstacle2D(new Pos2D(2, 0), new Pos2D(3, 8)));
        
        world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13)));
//        world.addRegion(new Obstacle2D(new Pos2D(2, 15), new Pos2D(4, 20)));
        
        
//        world.addRegion(new Obstacle2D(new Pos2D(6, 0), new Pos2D(8, 3)));
        world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20)));
        
        world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13)));
//        world.addRegion(new Obstacle2D(new Pos2D(10, 15), new Pos2D(12, 20)));
        
//        world.addRegion(new SpeedLimitRegion2D(new Pos2D(0, 0), new Pos2D(40, 20), 1));

        
        
//        world.addObstacle(new Obstacle2D(new Pos2D(9, 9), new Pos2D(20, 10)));


        Vehicle vehicle = new Vehicle(2, 5);
        Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 1), new Pos2D(38, 10), 19, 200);

        
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
//            System.out.println("t1: " + String.valueOf(result.t1[0]));
//            System.out.println("t2: " + String.valueOf(result.t2[0]));
//            System.out.println("q1: " + String.valueOf(result.q1[0]));
//            System.out.println("w1: " + String.valueOf(result.w1[0]));
            ResultWindow test = new ResultWindow(result, scen, totalTime);
            test.setVisible(true);
        } catch (IloException e) {
            e.printStackTrace();
        }
        solver.end();
        
    }

}
