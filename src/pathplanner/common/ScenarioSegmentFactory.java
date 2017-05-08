package pathplanner.common;

import pathplanner.preprocessor.PathSegment;


public class ScenarioSegmentFactory {
    public Scenario scenario;
    public Pos2D goal;
    public int fps;
    public double positionTolerance;
    public double maxTime;
    public PathSegment pathSegment;
    
    public Pos2D startPos = null;
    public Pos2D startVel = null;
    public Pos2D startAcc = new Pos2D(0, 0);
    public Pos2D goalVel = null;
    public boolean isFinal = false;
    public double maxGoalVel = Double.NaN;

    public ScenarioSegmentFactory(Scenario scenario, 
            Pos2D goal, 
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
