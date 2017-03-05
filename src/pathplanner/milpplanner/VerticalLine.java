package pathplanner.milpplanner;

import java.util.List;
import java.util.Map;

import pathplanner.common.Scenario;
import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;


public class VerticalLine extends Line {
    public final double x;
    public final boolean left;
    
    public VerticalLine(double x, boolean left){
        this.x = x;
        this.left = left;
    }

    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize, List<IloIntVar> slackVars) throws IloException {
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
