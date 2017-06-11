package pathplanner.common;



/**
 * A scenario, which is a trajectory planning problem. It contains a world (with obstacles), a vehicle, start position and velocity, and goal position and velocity
 *
 */
public class Scenario {
    public final World2D world;
    public final Vehicle vehicle;
    public Vector2D startPos;
    public Vector2D startVel;
    public Vector2D goal;
    public Vector2D goalVel;

    public Scenario(World2D world, Vehicle vehicle, Vector2D startPos, Vector2D startVel, Vector2D goal, Vector2D goalVel){
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
    }

}
