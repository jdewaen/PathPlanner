package pathplanner.common;

import pathplanner.preprocessor.PathSegment;


public class ScenarioSegmentFactory {
    public Scenario scenario;
    public Vector2D goal;
    public int fps;
    public double positionTolerance;
    public double maxTime;
    public PathSegment pathSegment;
    
    public Vector2D startPos = null;
    public Vector2D startVel = null;
    public Vector2D startAcc = null;
    public Vector2D goalVel = null;
    public boolean isFinal = false;
    public double maxGoalVel = Double.NaN;

    public ScenarioSegmentFactory(Scenario scenario, 
            Vector2D goal, 
            int fps, 
            double positionTolerance, 
            double maxTime, 
            PathSegment pathSegment){
        this.scenario = scenario;
        this.goal = goal;
        this.fps = fps;
        this.positionTolerance = positionTolerance;
        this.maxTime = maxTime;
        this.pathSegment = pathSegment;
    }

    public ScenarioSegment build(){
        return new ScenarioSegment(
                scenario.world,
                scenario.vehicle,
                startPos,
                startVel,
                startAcc,
                goal,
                goalVel,
                maxGoalVel,
                maxTime,
                fps,
                pathSegment,
                positionTolerance * scenario.vehicle.size,
                isFinal);
    }
}
