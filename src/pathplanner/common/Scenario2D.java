package pathplanner.common;


public class Scenario2D {
    public final World2D world;
    public final Vehicle vehicle;
    public final Pos2D startPos;
    public final Pos2D goal;
    public final double maxTime;
    public final int timeSteps;
    public final double deltaT;
    
    public Scenario2D(World2D world, Vehicle vehicle, Pos2D startPos, Pos2D goal, double maxTime, int timeSteps){
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
        this.goal = goal;
        this.maxTime = maxTime;
        this.timeSteps = timeSteps;
        this.deltaT = maxTime / timeSteps;
    }

}
