package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;


public class RegularLine extends Line {
    public final double a;
    public final double b;
    public final boolean above;
    
    
    protected RegularLine(double a, double b, boolean above){
        this.a = a;
        this.b = b;
        this.above = above;
    }


    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, IloCplex cplex) throws IloException {
        if(above){
            return cplex.le(b, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }else{
            return cplex.ge(b, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }
    }

}
