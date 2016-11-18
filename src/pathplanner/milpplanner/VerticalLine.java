package pathplanner.milpplanner;

import pathplanner.common.Scenario2D;
import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;


public class VerticalLine extends Line {
    public final double x;
    public final boolean left;
    
    protected VerticalLine(double x, boolean left){
        this.x = x;
        this.left = left;
    }

    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario2D scenario, IloCplex cplex) throws IloException {
        if(left){
            return cplex.le(vars.posX[t], x - scenario.vehicle.size);
        }else{
            return cplex.ge(vars.posX[t], x + scenario.vehicle.size);
        }
        
    }

}
