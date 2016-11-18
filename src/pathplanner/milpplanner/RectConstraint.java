package pathplanner.milpplanner;

import pathplanner.common.Region2D;
import pathplanner.common.Scenario2D;
import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;


public class RectConstraint implements ObstacleConstraint{
    
    Region2D region;
    
    protected RectConstraint(Region2D region){
        this.region = region;
    }

    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario2D scenario, IloCplex cplex)
            throws IloException {
        int largeNum = 99999;
        double buffer = scenario.vehicle.size;
        IloIntVar[] slack = cplex.intVarArray(4, 0, 1);;
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
