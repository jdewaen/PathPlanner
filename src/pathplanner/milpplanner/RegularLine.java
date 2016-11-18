package pathplanner.milpplanner;

import pathplanner.common.Scenario2D;
import pathplanner.common.Vehicle;
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
    public IloConstraint getConstraint(SolutionVars vars, int t,  Scenario2D scenario, IloCplex cplex) throws IloException {
        
        double alpha = Math.PI / 2 - Math.atan(-1/a);
        double diff = scenario.vehicle.size / Math.cos(alpha);
        if(above){
            return cplex.le(b - diff, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }else{
            return cplex.ge(b + diff, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }
    }

}
