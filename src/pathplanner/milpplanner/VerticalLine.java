package pathplanner.milpplanner;

import pathplanner.common.Scenario;
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
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize) throws IloException {
        double buffer;
        
        if (ignoreSize){
        	buffer = 0;
        }else{
        	buffer = scenario.vehicle.size;	
        }
    	
    	if(left){
            return cplex.le(vars.posX[t], x - buffer);
        }else{
            return cplex.ge(vars.posX[t], x + buffer);
        }
    }

}
