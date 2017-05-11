package test;

import java.util.Arrays;

import pathplanner.ScenarioFactory;
import pathplanner.common.Obstacle2DB;
import pathplanner.common.ObstacleImporter;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;


public abstract class Scenarios {
    
    public static ScenarioFactory benchmarkSmall(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Pos2D(25, 20));
        world.addObstacle(new Obstacle2DB(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(6, 5), new Pos2D(8, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(18, 0), new Pos2D(20, 13)));
 
        Pos2D start = new Pos2D(1, 1);
        Pos2D goal = new Pos2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory benchmarkLarge(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Pos2D(40, 20));
        world.addObstacle(new Obstacle2DB(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(6, 5), new Pos2D(8, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(18, 0), new Pos2D(20, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(22, 5), new Pos2D(24, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(26, 0), new Pos2D(28, 13)));
        world.addObstacle(new Obstacle2DB(new Pos2D(30, 5), new Pos2D(32, 20)));
        world.addObstacle(new Obstacle2DB(new Pos2D(34, 0), new Pos2D(36, 13)));
 
        Pos2D start = new Pos2D(1, 1);
        Pos2D goal = new Pos2D(38, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory sanFranciscoSmall(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Pos2D(-122.431704, 37.749849));
        
        Pos2D start = new Pos2D(186, 102);
        Pos2D goal = new Pos2D(918, 963);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory sanFranciscoLarge(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Pos2D(3000, 3000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Pos2D(-122.431704, 37.749849));
        
        Pos2D start = new Pos2D(2933, 230);
        Pos2D goal = new Pos2D(188, 2909);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;  
    }
    
    public static ScenarioFactory sanFranciscoSmallAlternate(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Pos2D(-122.431704, 37.749849));
        Pos2D start = new Pos2D(19, 893);
        Pos2D goal = new Pos2D(986, 10);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory leuvenSmall(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromKML(world, "data/GRBGebL1D2_173_174.kml", new Pos2D(4.695625, 50.875785), true);
        Pos2D start = new Pos2D(19, 893);
        Pos2D goal = new Pos2D(950, 133);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory leuvenSmallAlternate(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Pos2D(1000, 1000));
        ObstacleImporter.importFromKML(world, "data/GRBGebL1D2_173_174.kml", new Pos2D(4.695625, 50.875785), true);
        Pos2D start = new Pos2D(231, 186);
        Pos2D goal = new Pos2D(791, 817);
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory leuvenLarge(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Pos2D(2500, 2500));
        ObstacleImporter.importMultipleKML(world, Arrays.asList(
                "data/GRBGebL1D2_172_173.kml",
                "data/GRBGebL1D2_172_174.kml",
                "data/GRBGebL1D2_172_175.kml",
                "data/GRBGebL1D2_173_173.kml",
                "data/GRBGebL1D2_173_174.kml",
                "data/GRBGebL1D2_173_175.kml",
                "data/GRBGebL1D2_174_173.kml",
                "data/GRBGebL1D2_174_174.kml",
                "data/GRBGebL1D2_174_175.kml"
                
                ), new Pos2D(4.681767, 50.867162), true);
        Pos2D start = new Pos2D(2389, 246);
        Pos2D goal = new Pos2D(580, 2332);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory singleBlock(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Pos2D(40, 20));
        world.addObstacle(new Obstacle2DB(new Pos2D(10, 0), new Pos2D(30, 13)));

 
        Pos2D start = new Pos2D(1, 2);
        Pos2D goal = new Pos2D(37, 18);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory wallSkip(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 10, 0.5);        
 
        World2D world = new World2D(new Pos2D(40, 20));

        world.addObstacle(new Obstacle2DB(new Pos2D(23, 5), new Pos2D(24, 15)));
 
        Pos2D start = new Pos2D(1, 10);
        Pos2D goal = new Pos2D(38, 10);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory cornerSkip(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 10, 0.5);        
        
        World2D world = new World2D(new Pos2D(40, 20));

        world.addObstacle(new Obstacle2DB(new Pos2D(23, 15), new Pos2D(20, 0), new Pos2D(26, 1)));
 
        Pos2D start = new Pos2D(1, 10);
        Pos2D goal = new Pos2D(38, 10);

        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;   
    }
    
    public static ScenarioFactory octagon(){
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
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory spiral(){
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
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;     
    }
    
    public static ScenarioFactory airplane(){
        Vehicle vehicle = new Vehicle(2, 2.8, Double.NaN, 0.5);        

        World2D world = new World2D(new Pos2D(20, 10));


        Pos2D start = new Pos2D(1, 3);
        Pos2D goal = new Pos2D(1, 3);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.startVel = new Pos2D(2.8, 0);
        scenFact.goalVel = new Pos2D(-2.8, 0);
        
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory maxSpeed(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        

        World2D world = new World2D(new Pos2D(15, 15));


        Pos2D start = new Pos2D(1, 14);
        Pos2D goal = new Pos2D(14, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;   
    }
    
//    public static Scenario (){
//        
//        ScenarioFactory scenFact = new ScenarioFactory();
//        Scenario scenario = scenFact.build(world, vehicle, start, goal);
//        return scenario;     
//    }
}
