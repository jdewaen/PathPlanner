package pathplanner.common;

import java.io.Serializable;
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


public class ScenarioSegment implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = 4122882662840482650L;
    public final Vector2D startPos;
    public final Vector2D startVel;
    public final Vector2D startAcc;
    public final Vector2D goal;
    public final Vector2D goalVel;
    public final double maxTime;
    public final int timeSteps;
    public final double deltaT;
    public final Set<ObstacleConstraint> activeSet = new HashSet<ObstacleConstraint>();
    public List<Vector2D> activeRegion = null;
    public final PathSegment path;
    public final double positionTolerance;
    public final double maxSpeed;
    public final double maxGoalVel;
    public final Vehicle vehicle;
    public final boolean isFinal;
    public final int fps;
    public BoundsSolverDebugData boundsDebugData = null;
    
    public final double startingGrow = 2.1;
    
    public ScenarioSegment(World2D world, Vehicle vehicle, Vector2D startPos, Vector2D startVel, Vector2D startAcc,
            Vector2D goal, Vector2D goalVel, double maxGoalVel, double maxTime, int fps, PathSegment path, double positionTolerance, boolean isFinal){
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
            this.maxGoalVel = Double.POSITIVE_INFINITY;
        }

        this.vehicle = vehicle;
        this.isFinal = isFinal;
    }
    
    public long generateActiveSet(Scenario scenario, BoundsSolverConfig boundsConfig){
        boundsDebugData = new BoundsSolverDebugData();
        List<Vector2D> startingArea = getStartingArea(boundsConfig);
        List<Vector2D> grownConvex = GeometryToolbox.growPolygon(startingArea, boundsConfig.convexGrowMultiplier);
        boundsDebugData.seed = grownConvex;
        Set<Obstacle2D> activeObstacles = new HashSet<Obstacle2D>();
        for(Obstacle2D region : scenario.world.getObstacles()){
            if(GeometryToolbox.overlapsObstacle(grownConvex, region.shape)){
                activeSet.add(PolygonConstraint.fromRegion(region));
                activeObstacles.add(region);
            }
        }
        Set<Obstacle2D> inactiveObstacles = scenario.world.getObstacles().stream()
                .filter(obs -> !activeObstacles.contains(obs))
                .collect(Collectors.toSet());
        BoundsSolver regionSolver = new BoundsSolver(scenario, boundsConfig);

        if(boundsConfig.verbose) System.out.print("Starting BoundsSolver... ");
        long start = System.currentTimeMillis();
        activeRegion = regionSolver.solve(
                startPos.middleBetween(goal), 
                inactiveObstacles, 
                path.getDistance(),  
                startingArea,
                grownConvex,
                boundsDebugData);
        long millis = System.currentTimeMillis() - start;
        if(boundsConfig.verbose) System.out.println("DONE!");
        
        for(int i = 0; i < activeRegion.size(); i++){
            Vector2D first = activeRegion.get(i);
            Vector2D second = activeRegion.get((i + 1) % activeRegion.size());
            Vector2D delta = second.minus(first);
            if(delta.x != 0){
                double a = delta.y / delta.x;
                double b = first.y - a * first.x;
                activeSet.add(new RegularLine(a, b, (delta.x > 0)));
            }else{
               boolean left = delta.y > 0;
               activeSet.add(new VerticalLine(first.x, left));
            }

        }
        return millis;
        
    }
    
    public List<Vector2D> getStartingArea(BoundsSolverConfig boundsConfig){
        
        List<Vector2D> positions = path.toIndividualPositions().stream()
                .flatMap(pos -> GeometryToolbox.approximateCircle(pos, vehicle.size * startingGrow, boundsConfig.initRegionVertices, true).stream())
                .collect(Collectors.toList());
        
        
        positions.addAll(GeometryToolbox.approximateCircle(startPos, vehicle.size * startingGrow, boundsConfig.initRegionVertices, true));
        
        
        if(boundsConfig.useStopPoints){
            positions.addAll(GeometryToolbox.approximateCircle(getStopPoint(startPos, startVel), vehicle.size * startingGrow, boundsConfig.initRegionVertices, true));
            Vector2D maxFinishVelocity = path.getFinishVector().multiply(Math.min(maxGoalVel, vehicle.maxSpeed));
            positions.addAll(GeometryToolbox.approximateCircle(getStopPoint(goal, maxFinishVelocity), vehicle.size * startingGrow, boundsConfig.initRegionVertices, true));   
        }
        
        
        List<Vector2D> convex = GeometryToolbox.quickHull(positions);
        return convex;
    }
    
    private Vector2D getStopPoint(Vector2D pos, Vector2D vel){
        double dist = vehicle.getAccDist(vel.length());
        return pos.plus(vel.normalize().multiply(dist));
    }
    
    public int obstacleCount(){
        return (int) activeSet.stream().filter(c -> c instanceof PolygonConstraint).count();
    }
    
    public int edgeCount(){
        return (int) activeSet.stream().filter(c -> c instanceof PolygonConstraint)
                .flatMap(obs -> ((PolygonConstraint) obs).region.getVertices().stream())
                .count();
    }
}
