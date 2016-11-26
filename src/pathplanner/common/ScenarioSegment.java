package pathplanner.common;

import java.util.HashSet;
import java.util.Set;

import pathplanner.milpplanner.Line;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.RectConstraint;


public class ScenarioSegment {
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public final double maxTime;
    public final int timeSteps;
    public final double deltaT;
    public final Set<ObstacleConstraint> activeSet = new HashSet<ObstacleConstraint>();
    public final Set<Region2D> activeRegions;
    
    public ScenarioSegment(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D startVel, 
            Pos2D goal, Pos2D goalVel, double maxTime, int timeSteps, Set<Region2D> regions){
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
        this.startPos = startPos;
        this.startVel = startVel;
        this.goal = goal;
        this.goalVel = goalVel;
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        this.deltaT = maxTime / timeSteps;
        this.activeRegions = regions;
    }
    
    public void generateActiveSet(World2D world) throws Exception{
        for(Region2D region : world.getRegions()){
            if(activeRegions.contains(region) || region.intersects(startPos, goal)){
                activeSet.add(RectConstraint.fromRegion(region));
            }else{
                Line line = Line.fromRegion(region, startPos, goal);
                if(line != null){
                    activeSet.add(line);
                }
            }
        }
    }

}
