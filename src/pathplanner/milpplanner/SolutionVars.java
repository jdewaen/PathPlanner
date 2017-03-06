package pathplanner.milpplanner;

import ilog.concert.IloIntVar;
import ilog.concert.IloNumVar;

import java.util.List;
import java.util.Map;

public class SolutionVars {
    public IloNumVar[] posX;
    public IloNumVar[] posY;
    
    public IloNumVar[] velX;
    public IloNumVar[] velY;
    
    public IloNumVar[] absVelX;
    public IloNumVar[] absVelY;
    
    public IloNumVar[] verticalThrottle;
    public IloNumVar[] horizontalThrottle;
    
    public IloNumVar[] fin;
    public IloNumVar[] cfin;
    
    public IloNumVar[] time;
    
    public Map<PolygonConstraint, Map<Integer, List<IloIntVar>>> slackVars;

    

}
