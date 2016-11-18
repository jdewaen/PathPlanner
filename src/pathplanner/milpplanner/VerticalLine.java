package pathplanner.milpplanner;

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
    public IloConstraint getConstraint(SolutionVars vars, int t, IloCplex cplex) throws IloException {
        if(left){
            return cplex.le(vars.posX[t], x);
        }else{
            return cplex.ge(vars.posX[t], x);
        }
        
    }

}
