package pathplanner;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import pathplanner.common.GeometryToolbox;
import pathplanner.common.Obstacle2DB;
import pathplanner.common.ObstacleImporter;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.Solution;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;
import pathplanner.preprocessor.CheckpointGenerator;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.FixedAStar;
import pathplanner.preprocessor.Node;
import pathplanner.preprocessor.PathSegment;
import pathplanner.ui.ResultWindow;

public class Main {
	
    public static Scenario generateSparseScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));
        world.addObstacle(new Obstacle2DB(new Pos2D(10, 0), new Pos2D(30, 13)));

 
        Pos2D start = new Pos2D(1, 2);
        Pos2D goal = new Pos2D(37, 18);

                
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
        
        return scenario;
    }	
       
    public static Scenario generateBenchmarkScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));
//        world.addObstacle(new Obstacle2DB(Arrays.asList(new Pos2D(2, 0),
//                new Pos2D(4, 0),
//                new Pos2D(3.5, 12),
//                new Pos2D(2.5, 12))
//        ));
        world.addObstacle(new Obstacle2DB(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(6, 5), new Pos2D(8, 20))); //12 5
        world.addObstacle(new Obstacle2DB(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(18, 0), new Pos2D(20, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(22, 5), new Pos2D(24, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(26, 0), new Pos2D(28, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(30, 5), new Pos2D(32, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(34, 0), new Pos2D(36, 13)));
 
        Pos2D start = new Pos2D(1, 1);
        Pos2D goal = new Pos2D(38, 1);

                
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
        
        return scenario;
    }
    
    public static Scenario generateSkipScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 10, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));

        world.addObstacle(new Obstacle2DB(new Pos2D(23, 5), new Pos2D(24, 15)));
 
        Pos2D start = new Pos2D(1, 10);
        Pos2D goal = new Pos2D(38, 10);

                
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
        
        return scenario;
    }  
    
    public static Scenario generateCornerSkipScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 10, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));

        world.addObstacle(new Obstacle2DB(new Pos2D(23, 15), new Pos2D(20, 0), new Pos2D(26, 0)));
 
        Pos2D start = new Pos2D(1, 10);
        Pos2D goal = new Pos2D(38, 10);

                
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
        
        return scenario;
    }  
    
    public static Scenario generateOctagonScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(5, Double.NaN, 5, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));

        Obstacle2DB obs = new Obstacle2DB(
                new Pos2D(12, 9),
                new Pos2D(12, 11),
                new Pos2D(14, 13),
                new Pos2D(16, 13),
                new Pos2D(18, 11),
                new Pos2D(18, 9),
                new Pos2D(16, 7),
                new Pos2D(14, 7)
                );
        
        world.addObstacle(obs);
         
        Pos2D start = new Pos2D(1, 10);
        Pos2D goal = new Pos2D(38, 10);

                
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
        
        return scenario;
    }  
    
    public static Scenario generateBenchmarkScenario2() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));
        world.addObstacle(new Obstacle2DB(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(6, 12), new Pos2D(8, 20))); //12 5
        world.addObstacle(new Obstacle2DB(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(18, 0), new Pos2D(20, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(22, 5), new Pos2D(24, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(26, 0), new Pos2D(28, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(30, 5), new Pos2D(32, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(34, 0), new Pos2D(36, 13)));
 
        Pos2D start = new Pos2D(1, 1);
        Pos2D goal = new Pos2D(37, 1);

                
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
        
        
        return scenario;
    }
    
    
    public static Scenario generateSpiralScenario() throws Exception{
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        

        World2D world = new World2D(new Pos2D(30, 30));
        world.addObstacle(new Obstacle2DB(new Pos2D(13, 12), new Pos2D(16, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(13, 12), new Pos2D(14, 18)));
        world.addObstacle(new Obstacle2DB(new Pos2D(13, 17), new Pos2D(21, 18)));
        world.addObstacle(new Obstacle2DB(new Pos2D(20, 7), new Pos2D(21, 18)));
        world.addObstacle(new Obstacle2DB(new Pos2D(8, 7), new Pos2D(21, 8)));
        world.addObstacle(new Obstacle2DB(new Pos2D(8, 7), new Pos2D(9, 23)));
        world.addObstacle(new Obstacle2DB(new Pos2D(8, 22), new Pos2D(26, 23)));
        world.addObstacle(new Obstacle2DB(new Pos2D(25, 2), new Pos2D(26, 23)));
        world.addObstacle(new Obstacle2DB(new Pos2D(3, 2), new Pos2D(26, 3)));
        world.addObstacle(new Obstacle2DB(new Pos2D(3, 2), new Pos2D(4, 28)));
        world.addObstacle(new Obstacle2DB(new Pos2D(3, 27), new Pos2D(26, 28)));

        Pos2D start = new Pos2D(15, 15);
        Pos2D goal = new Pos2D(28, 25);
        
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
                
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
        
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        

        World2D world = new World2D(new Pos2D(15, 15));


        Pos2D start = new Pos2D(1, 14);
        Pos2D goal = new Pos2D(14, 1);
        
        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, null);
        
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

    
    public static Scenario generateSFScenario1() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Pos2D(-122.431704, 37.749849));
//        ObstacleImporter.convertToKML("san_francisco.csv", "SF.kml");
        Pos2D start = new Pos2D(186, 102);
        Pos2D goal = new Pos2D(918, 963);
//        Pos2D goal = new Pos2D(321, 111);



        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
                
        return scenario;
    }
    
    public static Scenario generateSFScenario2() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Pos2D(200, 100));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Pos2D(-122.431704, 37.749849));
//        ObstacleImporter.convertToKML("san_francisco.csv", "SF.kml");
        Pos2D start = new Pos2D(170, 10);
//        Pos2D goal = new Pos2D(918, 963);
        Pos2D goal = new Pos2D(184, 29);
        // 47 49



        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(0, 0), 
                goal, new Pos2D(0, 0));
                
        return scenario;
    }
    
    public static Scenario generateSFScenario3() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Pos2D(-122.431704, 37.749849));
//        ObstacleImporter.convertToKML("san_francisco.csv", "SF.kml");
        Pos2D start = new Pos2D(19, 893); // 821 947
        Pos2D goal = new Pos2D(986, 10);
//        Pos2D goal = new Pos2D(321, 111);



        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(11.55, 0), 
                goal, new Pos2D(0, 0));
                
        return scenario;
    }
    
    public static Scenario generateLeuvenScenario1() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 0.5);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromKML(world, "data/GRBGebL1D2_173_174.kml", new Pos2D(4.695625, 50.875785), true);
//        ObstacleImporter.convertToKML("san_francisco.csv", "SF.kml");
        Pos2D start = new Pos2D(19, 893); // 821 947
        Pos2D goal = new Pos2D(950, 133);
//        Pos2D goal = new Pos2D(321, 111);



        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(11.55, 0), 
                goal, new Pos2D(0, 0));
                
        return scenario;
    }
    
    public static Scenario generateLeuvenScenario2() throws Exception{
        
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromKML(world, "data/GRBGebL1D2_173_174.kml", new Pos2D(4.695625, 50.875785), true);
//        ObstacleImporter.convertToKML("san_francisco.csv", "SF.kml");
        Pos2D start = new Pos2D(231, 186); // 821 947
        Pos2D goal = new Pos2D(791, 817);
//        Pos2D goal = new Pos2D(321, 111);



        Scenario scenario = new Scenario(world, vehicle, start, new Pos2D(11.55, 0), 
                goal, new Pos2D(0, 0));
                
        return scenario;
    }
    
    public static void main(String[] args) {
        long startTime = System.currentTimeMillis(); 
        double gridSize = 5;
//        double gridSize = 1;
        try {
            

//            Scenario scenario = generateSparseScenario();
        	
//            Scenario scenario = generateSpiralScenario();
//            Solution solution = loadSolution("spiral.dat");

//            Scenario scenario = generateBenchmarkScenario();
//            Solution solution = loadSolution("benchmark.dat");          

            
//            Scenario scenario = generateAirplaneScenario();
//        	Scenario scenario = generateMaxSpeedScenario();

            
//            Scenario scenario = generateCornerSkipScenario();
//            Scenario scenario = generateSkipScenario();

//            Scenario scenario = generateSFScenario1();
//            Scenario scenario = generateOctagonScenario();
            
            Scenario scenario = generateLeuvenScenario2();
            
            FixedAStar preprocessor = new FixedAStar(scenario);
            System.out.println("Waiting for A*");
            LinkedList<Node> prePath= preprocessor.solve(gridSize);
            CheckpointGenerator gen = new CheckpointGenerator(scenario);
//            double cornerMargin = 0.5; // 0.2  g:0.5
//            double approachMargin = 2.5; // 2.5 g:1.5
//            double tolerance = 0.05; // g -> reversed on path segment 15
            
            double maxTime = 5; // 0.2  g:0.5
            double approachMargin = 2.5; // 2.5 g:1.5
            double tolerance = 1; // g -> reversed on path segment 15
            List<CornerEvent> corners = gen.generateCornerEvents(prePath, gridSize, tolerance);
            List<PathSegment> filtered = gen.generateFromPath(prePath, gridSize, corners, approachMargin, maxTime);
            scenario.generateSegments(filtered);
            Solution solution = scenario.solve();
//            Solution solution = Solution.generateEmptySolution();
            
            //TODO: grow obstacles with vehicle --> fixes A*
            
            
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
        oos.close();
    }
    
    public static Solution loadSolution(String filename) throws ClassNotFoundException, IOException{
        FileInputStream fout = new FileInputStream(filename);
        ObjectInputStream oos = new ObjectInputStream(fout);
        Solution result = (Solution) oos.readObject();
        oos.close();
        return result;
    }
   

}
