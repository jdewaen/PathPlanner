package pathplanner.common;

import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.stream.Collectors;

import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.preprocessor.PathSegment;


public class Scenario {
    public final World2D world;
    public final Vehicle vehicle;
    public Pos2D startPos;
    public Pos2D startVel;
    public Pos2D goal;
    public Pos2D goalVel;
    public List<ScenarioSegment> segments;
    public static final double POSITION_TOLERANCE = 3;
    public static final double POSITION_TOLERANCE_FINAL = 0.1;
    public static final int FPS = 5;
    public final StatisticsTracker stats;

    public Scenario(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D startVel, Pos2D goal, Pos2D goalVel, StatisticsTracker stats){
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
        this.stats = stats;
    }
    
    public void  generateSingleSegment(int time) throws Exception{
        ScenarioSegment segment = new ScenarioSegment(world, vehicle, startPos, startVel, goal, goalVel, Double.NaN, time, time*FPS, null, vehicle.size * POSITION_TOLERANCE_FINAL, true); 
        segments = new ArrayList<ScenarioSegment>();
        segments.add(segment);
    }
    
    public Solution solveSingle() throws Exception{
        ScenarioSegment segment = segments.get(0);
        segment.activeSet.addAll(world.getObstacles().stream().map(obs -> new PolygonConstraint(obs)).collect(Collectors.toList()));
        CPLEXSolver solver = new CPLEXSolver(this, segment, null);
        solver.generateConstraints();
        solver.solve();
        Solution result = null;
        try {
            result = solver.getResults();
            return result;
        } catch (IloException e) {
            e.printStackTrace();
            throw new Exception();
        } finally{
            solver.end();
        }

    }



}
