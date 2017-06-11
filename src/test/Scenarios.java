package test;

import java.util.Arrays;

import pathplanner.ScenarioFactory;
import pathplanner.common.Obstacle2D;
import pathplanner.common.ObstacleImporter;
import pathplanner.common.Vector2D;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;


public abstract class Scenarios {
    
    public static ScenarioFactory benchmarkSmall(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(25, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(4, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 5), new Vector2D(8, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(10, 0), new Vector2D(12, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(14, 5), new Vector2D(16, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(18, 0), new Vector2D(20, 13)));
 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory benchmark0(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(25, 20));

 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory benchmark1(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(25, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(4, 13)));

 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory benchmark2(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(25, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(4, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 5), new Vector2D(8, 20)));

 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }  
    
    public static ScenarioFactory benchmark3(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(25, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(4, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 5), new Vector2D(8, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(10, 0), new Vector2D(12, 13)));

 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    } 
    
    public static ScenarioFactory benchmark4(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(25, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(4, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 5), new Vector2D(8, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(10, 0), new Vector2D(12, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(14, 5), new Vector2D(16, 20)));
 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(22, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    } 
    public static ScenarioFactory flatBenchmarkSmall(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(100, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(30, 6)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 14), new Vector2D(45, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(32, 0), new Vector2D(64, 6)));
        world.addObstacle(new Obstacle2D(new Vector2D(50, 14), new Vector2D(95, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(66, 0), new Vector2D(96, 6)));
 
        Vector2D start = new Vector2D(1, 10);
        Vector2D goal = new Vector2D(99, 10);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory flatBenchmarkDiagSmall(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(100, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(30, 6)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 14), new Vector2D(45, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(32, 0), new Vector2D(64, 6)));
        world.addObstacle(new Obstacle2D(new Vector2D(50, 14), new Vector2D(95, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(66, 0), new Vector2D(90, 6)));
 
        Vector2D start = new Vector2D(1, 18);
        Vector2D goal = new Vector2D(98, 2);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory benchmarkLarge(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(40, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 0), new Vector2D(4, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(6, 5), new Vector2D(8, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(10, 0), new Vector2D(12, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(14, 5), new Vector2D(16, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(18, 0), new Vector2D(20, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(22, 5), new Vector2D(24, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(26, 0), new Vector2D(28, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(30, 5), new Vector2D(32, 20)));
        world.addObstacle(new Obstacle2D(new Vector2D(34, 0), new Vector2D(36, 13)));
 
        Vector2D start = new Vector2D(1, 1);
        Vector2D goal = new Vector2D(38, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory sanFranciscoTiny(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Vector2D(500, 500));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Vector2D(-122.431704, 37.749849));
        
        Vector2D start = new Vector2D(186, 102);
        Vector2D goal = new Vector2D(452, 462);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory sanFranciscoSmall(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Vector2D(1000, 1000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Vector2D(-122.431704, 37.749849));
        
        Vector2D start = new Vector2D(186, 102);
        Vector2D goal = new Vector2D(918, 963);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory sanFranciscoLarge(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Vector2D(3000, 3000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Vector2D(-122.431704, 37.749849));
        
        Vector2D start = new Vector2D(2933, 230);
        Vector2D goal = new Vector2D(188, 2909);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;  
    }
    
    public static ScenarioFactory sanFranciscoSmallAlternate(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);        

        World2D world = new World2D(new Vector2D(1000, 1000));
        ObstacleImporter.importFromFile(world, "san_francisco.csv", new Vector2D(-122.431704, 37.749849));
        Vector2D start = new Vector2D(123, 968);
        Vector2D goal = new Vector2D(907, 7);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory leuvenSmall(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Vector2D(1000, 1000));
        ObstacleImporter.importFromKML(world, "data/GRBGebL1D2_173_174.kml", new Vector2D(4.695625, 50.875785), true);
        Vector2D start = new Vector2D(19, 893);
        Vector2D goal = new Vector2D(950, 133);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;
    }
    
    public static ScenarioFactory leuvenSmallAlternate(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Vector2D(1000, 1000));
        ObstacleImporter.importFromKML(world, "data/GRBGebL1D2_173_174.kml", new Vector2D(4.695625, 50.875785), true);
        Vector2D start = new Vector2D(231, 186);
        Vector2D goal = new Vector2D(791, 817);
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory leuvenLarge(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);        

        World2D world = new World2D(new Vector2D(2500, 2500));
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
                
                ), new Vector2D(4.681767, 50.867162), true);
        Vector2D start = new Vector2D(2389, 246);
        Vector2D goal = new Vector2D(580, 2332);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory singleBlock(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        
        
        World2D world = new World2D(new Vector2D(40, 20));
        world.addObstacle(new Obstacle2D(new Vector2D(10, 0), new Vector2D(30, 13)));

 
        Vector2D start = new Vector2D(1, 2);
        Vector2D goal = new Vector2D(37, 18);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory wallSkip(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 10, 0.5);        
 
        World2D world = new World2D(new Vector2D(40, 20));

        world.addObstacle(new Obstacle2D(new Vector2D(23, 5), new Vector2D(24, 15)));
 
        Vector2D start = new Vector2D(1, 10);
        Vector2D goal = new Vector2D(38, 10);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory cornerSkip(){
        Vehicle vehicle = new Vehicle(10, Double.NaN, 10, 0.5);        
        
        World2D world = new World2D(new Vector2D(40, 20));

        world.addObstacle(new Obstacle2D(new Vector2D(23, 15), new Vector2D(20, 0), new Vector2D(26, 1)));
 
        Vector2D start = new Vector2D(1, 10);
        Vector2D goal = new Vector2D(38, 10);

        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;   
    }
    
    public static ScenarioFactory octagon(){
        Vehicle vehicle = new Vehicle(5, Double.NaN, 5, 0.5);        
        
        World2D world = new World2D(new Vector2D(40, 20));

        Obstacle2D obs = new Obstacle2D(
                new Vector2D(12, 9),
                new Vector2D(12, 11),
                new Vector2D(14, 13),
                new Vector2D(16, 13),
                new Vector2D(18, 11),
                new Vector2D(18, 9),
                new Vector2D(16, 7),
                new Vector2D(14, 7)
                );
        
        world.addObstacle(obs);
         
        Vector2D start = new Vector2D(1, 10);
        Vector2D goal = new Vector2D(38, 10);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory spiral(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        

        World2D world = new World2D(new Vector2D(30, 30));
        world.addObstacle(new Obstacle2D(new Vector2D(13, 12), new Vector2D(16, 13)));
        world.addObstacle(new Obstacle2D(new Vector2D(13, 12), new Vector2D(14, 18)));
        world.addObstacle(new Obstacle2D(new Vector2D(13, 17), new Vector2D(21, 18)));
        world.addObstacle(new Obstacle2D(new Vector2D(20, 7), new Vector2D(21, 18)));
        world.addObstacle(new Obstacle2D(new Vector2D(8, 7), new Vector2D(21, 8)));
        world.addObstacle(new Obstacle2D(new Vector2D(8, 7), new Vector2D(9, 23)));
        world.addObstacle(new Obstacle2D(new Vector2D(8, 22), new Vector2D(26, 23)));
        world.addObstacle(new Obstacle2D(new Vector2D(25, 2), new Vector2D(26, 23)));
        world.addObstacle(new Obstacle2D(new Vector2D(3, 2), new Vector2D(26, 3)));
        world.addObstacle(new Obstacle2D(new Vector2D(3, 2), new Vector2D(4, 28)));
        world.addObstacle(new Obstacle2D(new Vector2D(3, 27), new Vector2D(26, 28)));

        Vector2D start = new Vector2D(15, 15);
        Vector2D goal = new Vector2D(28, 25);
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;     
    }
    
    public static ScenarioFactory airplane(){
        Vehicle vehicle = new Vehicle(2, 2.8, Double.NaN, 0.5);        

        World2D world = new World2D(new Vector2D(20, 10));


        Vector2D start = new Vector2D(1, 3);
        Vector2D goal = new Vector2D(1, 3);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.startVel = new Vector2D(2.8, 0);
        scenFact.goalVel = new Vector2D(-2.8, 0);
        
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;    
    }
    
    public static ScenarioFactory maxSpeed(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);        

        World2D world = new World2D(new Vector2D(15, 15));


        Vector2D start = new Vector2D(1, 14);
        Vector2D goal = new Vector2D(14, 1);
        
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.goal = goal;
        
        return scenFact;   
    }
    
    public static ScenarioFactory cornerTooSharp(){
        Vehicle vehicle = new Vehicle(3, Double.NaN, 5, 0.5);        

        World2D world = new World2D(new Vector2D(35, 30));
        world.addObstacle(new Obstacle2D(new Vector2D(4, 22), new Vector2D(27, 24.1)));
        world.addObstacle(new Obstacle2D(new Vector2D(2, 22), new Vector2D(4, 28)));
        world.addObstacle(new Obstacle2D(new Vector2D(4, 26), new Vector2D(31, 28)));
        world.addObstacle(new Obstacle2D(new Vector2D(29, 17), new Vector2D(31, 28)));
        world.addObstacle(new Obstacle2D(new Vector2D(25, 17), new Vector2D(27, 24.1)));


        Vector2D start = new Vector2D(5, 25);
        Vector2D startVel = new Vector2D(4.5, 0);
        Vector2D goal = new Vector2D(28, 1);
        ScenarioFactory scenFact = new ScenarioFactory();
        scenFact.world = world;
        scenFact.vehicle = vehicle;
        scenFact.start = start;
        scenFact.startVel = startVel;
        scenFact.goal = goal;
        
        return scenFact;     
    }
    
}
