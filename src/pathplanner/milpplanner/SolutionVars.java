package pathplanner.milpplanner;

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
    

}
