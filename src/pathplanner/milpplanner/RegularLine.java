package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import pathplanner.common.Scenario;


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
    public IloConstraint getConstraint(SolutionVars vars, int t,  Scenario scenario, IloCplex cplex, boolean ignoreSize) throws IloException {
        
        double diff;
        
        if (ignoreSize){
        	diff = 0;
        }else{
            double alpha = Math.PI / 2 - Math.atan(-1/a);
            diff = scenario.vehicle.size / Math.cos(alpha);
            }
        if(above){
            return cplex.le(cplex.sum(b - diff, cplex.prod(vars.fin[t], LARGE_NUM)), cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }else{
            return cplex.ge(cplex.sum(b + diff, cplex.prod(vars.fin[t], LARGE_NUM)), cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }
    }
}
