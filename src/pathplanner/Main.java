package pathplanner;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.sql.Savepoint;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import pathplanner.common.*;
import pathplanner.preprocessor.CheckpointGenerator;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.FixedAStar;
import pathplanner.preprocessor.Node;
import pathplanner.preprocessor.PathSegment;
import pathplanner.ui.ResultWindow;

public class Main {
       
    public static Scenario generateBenchmarkScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));
        world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(6, 12), new Pos2D(8, 20))); //12 5
        world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(22, 5), new Pos2D(24, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(26, 0), new Pos2D(28, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(30, 5), new Pos2D(32, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(34, 0), new Pos2D(36, 13)));
 
        Pos2D start = new Pos2D(1, 1);
        Pos2D goal = new Pos2D(37, 1);

        
//        List<Pos2D> checkpoints = benchmarkCheckpoints();
        
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
//        scenario.generateSegments(checkpoints);
        
        return scenario;
    }
    
    
    public static Scenario generateSpiralScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        

        World2D world = new World2D(new Pos2D(30, 30));
        world.addRegion(new Obstacle2D(new Pos2D(13, 12), new Pos2D(16, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(13, 13), new Pos2D(14, 21)));
        world.addRegion(new Obstacle2D(new Pos2D(13, 20), new Pos2D(21, 21)));
        world.addRegion(new Obstacle2D(new Pos2D(20, 7), new Pos2D(21, 21)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 7), new Pos2D(21, 8)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 7), new Pos2D(9, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 22), new Pos2D(26, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(25, 2), new Pos2D(26, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 2), new Pos2D(26, 3)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 2), new Pos2D(4, 28)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 27), new Pos2D(26, 28)));

//        List<Pos2D> checkpoints = spiralCheckpoints();
        Pos2D start = new Pos2D(15, 15);
        Pos2D goal = new Pos2D(28, 25);
        
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
//        scenario.generateSegments(checkpoints);
        
        return scenario;
    }
    
    public static Scenario generateAirplaneScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(2, 2.8, Double.NaN, 0.5);        

        World2D world = new World2D(new Pos2D(20, 10));


        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        checkpoints.add(new Pos2D(1, 3));
        checkpoints.add(new Pos2D(10, 3));
        checkpoints.add(new Pos2D(1, 3));

        Scenario scenario = new Scenario(world, vehicle, checkpoints.get(0), new Pos2D(2.8, 0), 
                checkpoints.get(checkpoints.size() - 1), new Pos2D(-2.8, 0));
        
//        scenario.generateSegments(checkpoints);
        
        return scenario;
    }
    
    public static Scenario generateMaxSpeedScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 20, 0.5);        

        World2D world = new World2D(new Pos2D(15, 15));


        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        checkpoints.add(new Pos2D(1, 1));
        checkpoints.add(new Pos2D(14, 14));

        Scenario scenario = new Scenario(world, vehicle, checkpoints.get(0), new Pos2D(0, 0), 
                checkpoints.get(checkpoints.size() - 1), null);
        
//        scenario.generateSegments(checkpoints);
        
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
        double gridSize = 0.5;
        try {
            
            
//            Scenario scenario = generateSpiralScenario();
//            Solution solution = loadSolution("spiral.dat");

            Scenario scenario = generateBenchmarkScenario();
//            Solution solution = loadSolution("benchmark.dat");          

            
//            Scenario scenario = generateAirplaneScenario();
//        	Scenario scenario = generateMaxSpeedScenario();

            FixedAStar preprocessor = new FixedAStar(scenario);
            LinkedList<Node> prePath= preprocessor.solve(gridSize);
            CheckpointGenerator gen = new CheckpointGenerator(scenario);
            double cornerMargin = 0.5;
            double approachMargin = 1;
            double tolerance = 1;
            List<CornerEvent> corners = gen.generateCornerEvents(prePath, gridSize, cornerMargin, tolerance);
            List<PathSegment> filtered = gen.generateFromPath(prePath, gridSize, corners, approachMargin);
            scenario.generateSegments(filtered);
            Solution solution = scenario.solve();
            
//            saveSolution(solution, "benchmark.dat");
            

            
            
            long endTime   = System.currentTimeMillis();
            double totalTime = endTime - startTime;
            totalTime /= 1000;

            System.out.println(String.valueOf(totalTime));
            ResultWindow test = new ResultWindow(solution, scenario, totalTime, prePath, PathSegment.toPositions(filtered), corners);
            test.setVisible(true);
        } catch (Exception e) {
            e.printStackTrace();
        }
        
    }
    
    
    public static void saveSolution(Solution sol, String filename) throws IOException{
        FileOutputStream fout = new FileOutputStream(filename);
        ObjectOutputStream oos = new ObjectOutputStream(fout);
        oos.writeObject(sol);
    }
    
    public static Solution loadSolution(String filename) throws ClassNotFoundException, IOException{
        FileInputStream fout = new FileInputStream(filename);
        ObjectInputStream oos = new ObjectInputStream(fout);
        Solution result = (Solution) oos.readObject();
        return result;
    }
   

}
