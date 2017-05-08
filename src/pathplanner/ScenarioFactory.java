package pathplanner;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;


public class ScenarioFactory {
    public Pos2D startVel = new Pos2D(0,0);
    public Pos2D goalVel = new Pos2D(0,0);
    
    
    public Scenario build(World2D world, Vehicle vehicle, Pos2D start, Pos2D goal){
        return new Scenario(
                world, 
                vehicle, 
                start, 
                startVel, 
                goal, 
                goalVel);
    }
}
