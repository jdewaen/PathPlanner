package pathplanner.common;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import pathplanner.milpplanner.Line;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.RectConstraint;


public class Scenario2D {
    public final World2D world;
    public final Vehicle vehicle;
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public final double maxTime;
    public final int timeSteps;
    public final double deltaT;
    public final Set<ObstacleConstraint> activeSet = new HashSet<ObstacleConstraint>();
    
    public Scenario2D(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D startVel, Pos2D goal, Pos2D goalVel, double maxTime, int timeSteps){
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
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        this.deltaT = maxTime / timeSteps;
    }
    
    public void generateActiveSet() throws Exception{
        for(Region2D region : world.getRegions()){
            if(region.intersects(startPos, goal)){
                activeSet.add(RectConstraint.fromRegion(region));
            }else{
                Line line = Line.fromRegion(region, startPos, goal);
                if(line != null){
                    activeSet.add(line);
                }
            }
        }
    }
    
    public static List<Scenario2D> generateScenarios(List<Pos2D> checkpoints, World2D world, Vehicle vehicle){
        List<Scenario2D> scenarios = new ArrayList<Scenario2D>();
        for( int i = 1; i < checkpoints.size(); i++){
            if( i != checkpoints.size() - 1){
                scenarios.add(new Scenario2D(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), null, 10, 100));
            }else{
                scenarios.add(new Scenario2D(world, vehicle, checkpoints.get(i - 1), null, checkpoints.get(i), new Pos2D(0, 0), 10, 100)); 
            }    
        } 
      
      
        try {
            for(Scenario2D scen : scenarios) scen.generateActiveSet();
        } catch (Exception e) {
            e.printStackTrace();
        }

        return scenarios;
    }

}
