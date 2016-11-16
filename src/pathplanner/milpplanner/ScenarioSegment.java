package pathplanner.milpplanner;

import java.util.HashSet;
import java.util.Set;

import pathplanner.common.*;


public class ScenarioSegment {
    public Set<Sphere2D> spheres = new HashSet<Sphere2D>();
    public Pos2D start;
    public Pos2D goal;
    
    public ScenarioSegment(Pos2D start, Pos2D goal) {
        this.start = start;
        this.goal = goal;
    }

}
