package pathplanner.common;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.milpplanner.RegularLine;
import pathplanner.milpplanner.VerticalLine;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.boundssolver.BoundsSolver;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfig;
import pathplanner.preprocessor.boundssolver.BoundsSolverDebugData;


public class ScenarioSegment {
    public final Pos2D startPos;
    public final Pos2D startVel;
    public final Pos2D startAcc;
    public final Pos2D goal;
    public final Pos2D goalVel;
    public final double maxTime;
    public final int timeSteps;
    public final double deltaT;
    public final Set<ObstacleConstraint> activeSet = new HashSet<ObstacleConstraint>();
    public List<Pos2D> activeRegion = null;
    public final PathSegment path;
    public final double positionTolerance;
    public final double maxSpeed;
    public final double maxGoalVel;
    public final Vehicle vehicle;
    public final boolean isFinal;
    public final int fps;
    public BoundsSolverDebugData boundsDebugData = null;
    
    public final double startingGrow = 2.1;
    
    public ScenarioSegment(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D startVel, Pos2D startAcc,
            Pos2D goal, Pos2D goalVel, double maxGoalVel, double maxTime, int fps, PathSegment path, double positionTolerance, boolean isFinal){
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
        this.startAcc = startAcc;
        this.goal = goal;
        this.goalVel = goalVel;
        this.maxTime = maxTime;
        this.fps = fps;
        this.deltaT = 1.0 / fps;
        this.timeSteps = (int) Math.ceil(maxTime * fps);
        this.path = path;
        this.positionTolerance = positionTolerance;
        this.maxSpeed = vehicle.maxSpeed;
        if(!Double.isNaN(maxGoalVel)){
            this.maxGoalVel = maxGoalVel;
        }else{
            if(path != null){
                this.maxGoalVel = path.goalVel;
            }else{
                this.maxGoalVel = vehicle.maxSpeed;
            }
            
        }
        this.vehicle = vehicle;
        this.isFinal = isFinal;
    }
    
    public void generateActiveSet(Scenario scenario, BoundsSolverConfig boundsConfig) throws Exception{
        boundsDebugData = new BoundsSolverDebugData();
        List<Pos2D> startingArea = getStartingArea();
        List<Pos2D> grownConvex = GeometryToolbox.growPolygon(startingArea, 0.1);
        boundsDebugData.seed = grownConvex;
        Set<Obstacle2DB> activeObstacles = new HashSet<Obstacle2DB>();
        for(Obstacle2DB region : scenario.world.getObstacles()){
            if(GeometryToolbox.overlapsObstacle(grownConvex, region.shape)){
                activeSet.add(PolygonConstraint.fromRegion(region));
                activeObstacles.add(region);
            }else{
//                Line line = Line.fromRegion(region, startPos, goal);
//                if(line != null){
//                    activeSet.add(line);
//                }
            }
        }
        Set<Obstacle2DB> inactiveObstacles = scenario.world.getObstacles().stream()
                .filter(obs -> !activeObstacles.contains(obs))
                .collect(Collectors.toSet());
//        System.out.println("start constructor");
        BoundsSolver regionSolver = new BoundsSolver(scenario, boundsConfig);

        if(boundsConfig.verbose) System.out.print("Starting BoundsSolver... ");
        activeRegion = regionSolver.solve(
                startPos.middleBetween(goal), 
                inactiveObstacles, 
                path.getDistance(),  
                startingArea,
                grownConvex,
                boundsDebugData);        
        if(boundsConfig.verbose) System.out.println("DONE!");
        
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
        List<Pos2D> positions = path.toIndividualPositions().stream()
                .flatMap(pos -> GeometryToolbox.approximateCircle(pos, vehicle.size * startingGrow, 6, true).stream())
                .collect(Collectors.toList());
        positions.addAll(GeometryToolbox.approximateCircle(startPos, vehicle.size * startingGrow, 6, true));
        positions.addAll(GeometryToolbox.approximateCircle(getStopPoint(startPos, startVel), vehicle.size * startingGrow, 6, true));
        Pos2D maxFinishVelocity = path.getFinishVector().multiply(Math.min(maxGoalVel, vehicle.maxSpeed));
        positions.addAll(GeometryToolbox.approximateCircle(getStopPoint(goal, maxFinishVelocity), vehicle.size * startingGrow, 6, true));
        List<Pos2D> convex = GeometryToolbox.quickHull(positions);
        return convex;
    }
    
    private Pos2D getStopPoint(Pos2D pos, Pos2D vel){
        double dist = vehicle.getAccDist(vel.length());
        return pos.plus(vel.normalize().multiply(dist));
    }

}
