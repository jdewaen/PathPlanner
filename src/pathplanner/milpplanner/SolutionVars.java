package pathplanner.milpplanner;

import java.util.HashMap;

import pathplanner.common.Region2D;
import ilog.concert.*;

public class SolutionVars {
    public IloNumVar[] posX;
    public IloNumVar[] posY;
    
    public IloNumVar[] velX;
    public IloNumVar[] velY;
    
    public IloNumVar[] verticalThrottle;
    public IloNumVar[] horizontalThrottle;
    
    public IloNumVar[] fin;
    public IloNumVar[] cfin;
    
    public IloNumVar[] time;
    
    public HashMap<Region2D, IloNumVar> checkpoints;
    public HashMap<Region2D, IloIntVar[]> checkpointsCounter;
    public HashMap<Region2D, IloIntVar[]> checkpointsChange;

    

}
