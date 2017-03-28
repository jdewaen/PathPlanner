package pathplanner.common;

import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import pathplanner.boundssolver.BoundsSolver;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.milpplanner.RegularLine;
import pathplanner.milpplanner.VerticalLine;
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
    public List<Pos2D> activeRegion = null;
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
        List<Pos2D> startingArea = getStartingArea();
        for(Obstacle2DB region : world.getObstacles()){
            if(path.obstacles.contains(region) || GeometryToolbox.overlapsObstacle(startingArea, region.shape)){
                activeSet.add(PolygonConstraint.fromRegion(region));
            }else{
//                Line line = Line.fromRegion(region, startPos, goal);
//                if(line != null){
//                    activeSet.add(line);
//                }
            }
        }
        System.out.println("start constructor");
        BoundsSolver regionSolver = new BoundsSolver(world,
                vehicle,
                startPos.middleBetween(goal), 
                activeSet.stream().filter(PolygonConstraint.class::isInstance).map(PolygonConstraint.class::cast)
                .map(cons -> cons.region).collect(Collectors.toSet()), 
                path.getDistance(), 
                Arrays.asList(startPos, goal),
                startingArea);
        System.out.println("start solve");
        activeRegion = regionSolver.solve();        
        System.out.println("done solve");
        
        for(int i = 0; i < activeRegion.size(); i++){
            Pos2D first = activeRegion.get(i);
            Pos2D second = activeRegion.get((i + 1) % activeRegion.size());
            Pos2D delta = second.minus(first);
            if(delta.x != 0){
                double a = delta.y / delta.x;
                double b = first.y - a * first.x;
                activeSet.add(new RegularLine(a, b, (delta.x > 0)));
            }else{
               boolean left = delta.y > 0;
               activeSet.add(new VerticalLine(first.x, left));
            }

        }

    }
    
    public List<Pos2D> getStartingArea(){
//        List<Pos2D> positions = path.toIndividualPositions().stream()
//                .flatMap(pos -> GeometryToolbox.approximateCircle(pos, vehicle.size * 2, 3).stream())
//                .collect(Collectors.toList());
        List<Pos2D> positions = path.toIndividualPositions(); // TODO: why is just vehicle size not good enough? 
//        positions.clear();
        positions.addAll(GeometryToolbox.approximateCircle(startPos, vehicle.size * 2, 6));
        positions.addAll(GeometryToolbox.approximateCircle(goal, vehicle.size * 2, 6));
//        positions.addAll(GeometryToolbox.approximateCircle(getStopPoint(startPos, startVel), vehicle.size * 2, 6));
        List<Pos2D> convex = GeometryToolbox.quickHull(positions);
        List<Pos2D> grownConvex = GeometryToolbox.growPolygon(convex, vehicle.size * 2);
        return grownConvex;
    }
    
    private Pos2D getStopPoint(Pos2D pos, Pos2D vel){
        double dist = vehicle.getAccDist(vel.length());
        return pos.plus(vel.normalize().multiply(dist));
    }

}
