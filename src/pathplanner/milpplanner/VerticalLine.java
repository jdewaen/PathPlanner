package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;

import java.util.List;

import pathplanner.common.Scenario;


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
    
    @Override
    public IloConstraint preventSkipping(IloCplex cplex, List<IloIntVar> last,
            List<IloIntVar> current) {
        return null;
    }
    
    @Override
    public IloConstraint preventCornerCutting(IloCplex cplex, List<IloIntVar> last,
            List<IloIntVar> current) {
        return null;
    }
}
