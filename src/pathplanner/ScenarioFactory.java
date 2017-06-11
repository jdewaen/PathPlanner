package pathplanner;
import pathplanner.common.Vector2D;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;


public class ScenarioFactory {
    public Vector2D startVel = new Vector2D(0,0);
    public Vector2D goalVel = new Vector2D(0,0);
    public World2D world = null;
    public Vehicle vehicle = null;
    public Vector2D start = null;
    public Vector2D goal = null;
    
    
    public Scenario build(){
        if(world == null) throw new IllegalStateException("Cannot build scenario: World is null");
        if(vehicle == null) throw new IllegalStateException("Cannot build scenario: Vehicle is null");
        if(start == null) throw new IllegalStateException("Cannot build scenario: Start position is null");
        if(goal == null) throw new IllegalStateException("Cannot build scenario: Goal position is null");
        return new Scenario(
                world, 
                vehicle, 
                start, 
                startVel, 
                goal, 
                goalVel);
    }
}
