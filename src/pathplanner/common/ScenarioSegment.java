package pathplanner.common;

import java.util.HashSet;
import java.util.Set;

import pathplanner.milpplanner.Line;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.RectConstraint;
import pathplanner.preprocessor.PathSegment;


public class ScenarioSegment {
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public final double maxTime;
    public final int timeSteps;
    public final double deltaT;
    public final Set<ObstacleConstraint> activeSet = new HashSet<ObstacleConstraint>();
    public final PathSegment path;
    public final double positionTolerance;
    public double maxSpeed;
    public double maxGoalVel;
    public Vehicle vehicle;
    
    public ScenarioSegment(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D startVel, 
            Pos2D goal, Pos2D goalVel, double maxGoalVel, double maxTime, int timeSteps, PathSegment path, double positionTolerance){
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
        this.path = path;
        this.positionTolerance = positionTolerance;
        this.maxSpeed = vehicle.maxSpeed;
        this.maxGoalVel = maxGoalVel;
        this.vehicle = vehicle;
    }
    
    public void generateActiveSet(World2D world) throws Exception{
        for(Region2D region : world.getRegions()){
            if(path.obstacles.contains(region) || region.intersects(startPos, goal, vehicle.size)){
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
