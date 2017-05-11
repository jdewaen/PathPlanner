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
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, CPLEXSolverConfig config, List<IloIntVar> slackVars) throws IloException {
        double buffer;
        
        if (config.ignoreVehicleSize){
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
