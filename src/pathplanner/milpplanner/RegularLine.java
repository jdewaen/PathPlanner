package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;

import java.util.List;

import pathplanner.common.Scenario;


public class RegularLine extends Line {
    /**
     * 
     */
    private static final long serialVersionUID = 2648636387559271010L;
    public final double a;
    public final double b;
    public final boolean above;
    
    
    public RegularLine(double a, double b, boolean above){
        this.a = a;
        this.b = b;
        this.above = above;
    }


    @Override
    public IloConstraint getConstraint(SolutionVars vars, int t,  Scenario scenario, IloCplex cplex, CPLEXSolverConfig config, List<IloIntVar> slackVars) throws IloException {
        
        double diff = 0;
        
        if (!config.ignoreVehicleSize){
            double alpha = Math.PI / 2 - Math.atan(-1/a);
            diff = Math.abs(scenario.vehicle.size / Math.cos(alpha));
        }
        try{
        if(above){
            return cplex.le(b + diff, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }else{
            return cplex.ge(b - diff, cplex.diff(vars.posY[t], cplex.prod(a, vars.posX[t])));
        }
        }catch(Exception e){
            System.out.println("bla");
            throw e;
        }
    }

}
