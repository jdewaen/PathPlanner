package pathplanner.common;

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
import pathplanner.preprocessor.Node;
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
    public List<List<Pos2D>> activeRegion = null;
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
        Rectangle2D startingArea = getStartingArea();
        for(Obstacle2DB region : world.getObstacles()){
            if(path.obstacles.contains(region) || region.shape.intersects(startingArea)){
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
                getPathAsList());
        
        List<Pos2D> test = Arrays.asList(
                new Pos2D(3.8309288897002776, -3.3444738636853515), 
                new Pos2D(1.5898573416340256, 8.411769262825505), 
                new Pos2D(-3.1524602821602272, 7.8954476159807205), 
                new Pos2D(-4.941127933660106, 6.5588757880525606), 
                new Pos2D(-4.691341903932967, -1.4517274816936836));
        regionSolver.overlapsObstacle(test);
        
        System.out.println("start solve");
        List<List<Pos2D>> tmpRegion = regionSolver.solve();
        List<Pos2D> points = QuickHull.quickHull(tmpRegion.stream().flatMap(l -> l.stream()).collect(Collectors.toList()));
        activeRegion = tmpRegion;
        System.out.println("done solve");
        
        
        
            for(int j = 0; j < points.size(); j++){
                Pos2D first = points.get(j);
                Pos2D second = points.get((j + 1) % points.size());
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
    
    public List<Pos2D> getPathAsList(){
        List<Pos2D> result = new ArrayList<Pos2D>();
        Node current = path.start;
        result.add(current.pos);
        do{
            current = current.getChild();
            result.add(current.pos);
        }while(current != path.end);
        
        return result;
    }
    
    public Rectangle2D getStartingArea(){
        List<Rectangle2D> rects = new ArrayList<Rectangle2D>();
        Node current = path.start;
        rects.add(BoundsSolver.pointToRect(current.pos, vehicle.size * 2));
        do{
            current = current.getChild();
            rects.add(BoundsSolver.pointToRect(current.pos, vehicle.size * 2));
        }while(current != path.end);
//        rects.add(BoundsSolver.pointToRect(startPos, vehicle.size * 2));
//        rects.add(BoundsSolver.pointToRect(goal, vehicle.size * 2));
        Pos2D[] bounds = BoundsSolver.rectsBoundingBox(rects);
        return new Rectangle2D.Double(bounds[0].x, bounds[0].y, bounds[1].x - bounds[0].x, bounds[1].y - bounds[0].y);
    }

}
