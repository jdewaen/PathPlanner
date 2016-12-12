package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;
import pathplanner.common.Region2D;
import pathplanner.common.Scenario;


public class RectConstraint implements ObstacleConstraint{
    
    public final Region2D region;
    
    protected RectConstraint(Region2D region){
        this.region = region;
    }

    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize)
            throws IloException {
        int largeNum = 99999;
        double buffer;
        
        if (ignoreSize){
        	buffer = 0;
        }else{
        	buffer = scenario.vehicle.size;	
        }
        
        IloIntVar[] slack = cplex.intVarArray(4, 0, 1);
        IloConstraint cons = cplex.le(cplex.sum(vars.posX[t], buffer), cplex.sum(region.bottomRightCorner.x, cplex.prod(largeNum, slack[0])));
        cons = cplex.and(cons, cplex.le(cplex.negative(cplex.sum(vars.posX[t], -buffer)), cplex.sum(-region.topLeftCorner.x, cplex.prod(largeNum, slack[1]))));
        cons = cplex.and(cons, cplex.le(cplex.sum(vars.posY[t], buffer), cplex.sum(region.bottomRightCorner.y, cplex.prod(largeNum, slack[2]))));
        cons = cplex.and(cons, cplex.le(cplex.negative(cplex.sum(vars.posY[t], -buffer)), cplex.sum(-region.topLeftCorner.y, cplex.prod(largeNum, slack[3]))));
        cons = cplex.and(cons, cplex.addLe(cplex.sum(slack), 3));
        return cons;

    }
    
    public static RectConstraint fromRegion(Region2D region){
        return new RectConstraint(region);
    }

}
