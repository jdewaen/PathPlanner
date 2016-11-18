package pathplanner;

import java.util.ArrayList;
import java.util.List;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {
       
    public static Scenario generateBenchmarkScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, 5, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));
        world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(22, 5), new Pos2D(24, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(26, 0), new Pos2D(28, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(30, 5), new Pos2D(32, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(34, 0), new Pos2D(36, 13)));
        
        List<Pos2D> checkpoints = benchmarkCheckpoints();
        
        Scenario scenario = new Scenario(world, vehicle, checkpoints.get(0), new Pos2D(0, 0), 
                checkpoints.get(checkpoints.size() - 1), new Pos2D(0, 0));
        
        scenario.generateSegments(checkpoints);
        
        return scenario;
    }
    
    
    public static Scenario generateSpiralScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, 5, 0.5);        

        World2D world = new World2D(new Pos2D(30, 30));
        world.addRegion(new Obstacle2D(new Pos2D(13, 12), new Pos2D(16, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(13, 12), new Pos2D(14, 18)));
        world.addRegion(new Obstacle2D(new Pos2D(13, 17), new Pos2D(21, 18)));
        world.addRegion(new Obstacle2D(new Pos2D(20, 7), new Pos2D(21, 18)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 7), new Pos2D(21, 8)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 7), new Pos2D(9, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 22), new Pos2D(26, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(25, 2), new Pos2D(26, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 2), new Pos2D(26, 3)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 2), new Pos2D(4, 28)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 27), new Pos2D(26, 28)));

        List<Pos2D> checkpoints = spiralCheckpoints();
        
        Scenario scenario = new Scenario(world, vehicle, checkpoints.get(0), new Pos2D(0, 0), 
                checkpoints.get(checkpoints.size() - 1), new Pos2D(0, 0));
        
        scenario.generateSegments(checkpoints);
        
        return scenario;
    }
    
    public static List<Pos2D> benchmarkCheckpoints(){
        
        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        checkpoints.add(new Pos2D(1, 1));
        checkpoints.add(new Pos2D(1, 8));
        checkpoints.add(new Pos2D(5, 8));
        checkpoints.add(new Pos2D(9, 8));
        checkpoints.add(new Pos2D(13, 8));
        checkpoints.add(new Pos2D(17, 8));
        checkpoints.add(new Pos2D(21, 8));
        checkpoints.add(new Pos2D(25, 8));
        checkpoints.add(new Pos2D(29, 8));
        checkpoints.add(new Pos2D(33, 8));
        checkpoints.add(new Pos2D(37, 8));
        checkpoints.add(new Pos2D(37, 1));

        return checkpoints;
    }
    
    public static List<Pos2D> spiralCheckpoints(){
        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        checkpoints.add(new Pos2D(15, 15));
        checkpoints.add(new Pos2D(15, 10));
        checkpoints.add(new Pos2D(10, 15));
        checkpoints.add(new Pos2D(16, 20));
        checkpoints.add(new Pos2D(23, 13));
        checkpoints.add(new Pos2D(16, 6));
        checkpoints.add(new Pos2D(6, 15));
        checkpoints.add(new Pos2D(17, 25));
        checkpoints.add(new Pos2D(28, 25));

        return checkpoints;
        

    }

    public static void main(String[] args) {
        long startTime = System.currentTimeMillis(); 
        try {
            
            
            Scenario scenario = generateSpiralScenario();
            Solution solution = scenario.solve();
            
            
            long endTime   = System.currentTimeMillis();
            double totalTime = endTime - startTime;
            totalTime /= 1000;
            ResultWindow test = new ResultWindow(solution, scenario, totalTime);
            test.setVisible(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
        
    }
   

}
