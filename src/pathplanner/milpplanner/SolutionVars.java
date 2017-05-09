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
    
    public IloNumVar[] accX;
    public IloNumVar[] accY;

    public IloNumVar[] jerkX;
    public IloNumVar[] jerkY;
    
    public IloNumVar[] absVelX;
    public IloNumVar[] absVelY;
    
    public IloNumVar[] absAccX;
    public IloNumVar[] absAccY;
    
    public IloNumVar[] absJerkX;
    public IloNumVar[] absJerkY;
    
    public IloIntVar[] fin;
    public IloIntVar[] cfin;
    
    public IloNumVar[] time;
    
//    public IloNumVar finishDotProduct;
    
    public Map<PolygonConstraint, Map<Integer, List<IloIntVar>>> slackVars;

    

}
